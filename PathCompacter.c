/*
   PathCompacter.c
   08/29/2015
   Author: Michael Casebolt, Brett Casebolt
*/

/*
The MIT License (MIT)

Copyright (c) 2015 Michael Casebolt and Brett Casebolt

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/

#include "PathCompacter.h"

#include <stdlib.h> // For memory management
#include <errno.h> // For the errno global and checking ENOMEM
#include <string.h> // For memmove and memcpy
#include <math.h> // For sqrt and fabs

// These are out of order for the purposes of struct packing.
typedef struct CompactPathSubproblemCall
   {
   DVector2D *pPointArray;
   DVector2D *pResultPointArray;
   int iPointsInCurrentPath;
   int iCopyFirstPoint;
   } CompactPathSubproblemCall;
   
typedef enum CompactPathResultCode
   {
   COMPACT_PATH_RESULT_CODE_DIVIDE,
   COMPACT_PATH_RESULT_CODE_LINEARIZE,
   COMPACT_PATH_RESULT_CODE_SOLVED
   } CompactPathResultCode;
   
CompactPathResultCode CompactPathSubproblemSolver(DVector2D *pPointArray, int iPointsInCurrentPath,
      DVector2D *pResultPointArray, int *piPointsInResultPath, int *piDivisionIndex,
      double dEpsilon);

// The call stack will start able to hold this many calls and grow by this amount whenever it needs
// to grow in size.
#define COMPACT_PATH_CALL_STACK_UNIT 2048

// callStackBase is a double pointer because realloc might move the base pointer.
static int CompactPathCallStackPush(CompactPathSubproblemCall **ppCallStackBase, int *piCallStackCapacity,
                                    int *piNumCallsInStack, CompactPathSubproblemCall *pCall)
   {
   // Check if the stack is full.
   if (*piNumCallsInStack >= *piCallStackCapacity)
      {
      // Grow the stack by a unit.
      *piCallStackCapacity += COMPACT_PATH_CALL_STACK_UNIT;
      *ppCallStackBase = realloc(*ppCallStackBase,
                                 sizeof(CompactPathSubproblemCall) * *piCallStackCapacity);
      if (errno || *ppCallStackBase == NULL)
         {
         // The stack can't grow.
         return 0; // Failure
         }
      }
      
      // Add the new call
      (*ppCallStackBase)[*piNumCallsInStack] = *pCall;
      ++(*piNumCallsInStack);
      
      return 1; // Success
   }

static int CompactPathCallStackPop(CompactPathSubproblemCall *pCallStackBase,
                                   int *piNumCallsInStack, CompactPathSubproblemCall *pPoppedCall)
   {
   // Check if there is a call to pop.
   if (*piNumCallsInStack > 0)
      {
      --(*piNumCallsInStack);
      *pPoppedCall = pCallStackBase[*piNumCallsInStack];
      return 1; // Success
      }
   else
      {
      // There are no calls to pop.
      return 0; // Failure
      }
   }

// This is the cleanup macro for the CompactPath function.
#define COMPACT_PATH_RETURN(iReturnValue)\
   errno = iOldErrno;\ // Restore the error code.
   free(pCallStackBase);\ // Free the only allocated memory.
   return iReturnValue;

#define FAILURE 0
#define SUCCESS 1
   
// This function iteratively simulates the recursive Ramer-Douglas-Peucker algorithm.
// https://en.wikipedia.org/wiki/Ramer%E2%80%93Douglas%E2%80%93Peucker_algorithm
// Please allocate the resultPointArray to be as large as the pointArray passed in.
// It would be a good idea to resize the allocated space for resultPointArray after this
// function returns using the value of pointsInResultPath.
// This algorithm works in-place. That is, you can use the same array for both pointArray
// and resultPointArray, keeping in mind that doing so will likely alter pointArray.
// Returns a true value (1) on successful completion, and returns a false value (0) otherwise.
// On failure, pointsInResultPath and the contents of resultPointArray are undefined.
int CompactPath(DVector2D *pPointArray, int iPointsInCurrentPath,
                DVector2D *pResultPointArray, int *piPointsInResultPath, double dEpsilon)
   {
   CompactPathSubproblemCall firstSubproblem, secondSubproblem;
   int iDivisionIndex; // Where should we split the problem into subproblems?
   CompactPathResultCode subproblemResultCode; // The status of the most recent subproblem call
   int iNumSolvedPoints = 0; // Keep track of how much of the result array is solved and in place.
   int iPointsInResultPath; // Number of valid points in the result array after a subproblem call
   
   // Save the old value of errno so that it can be restored before returning.
   int iOldErrno = errno;
   
   // Clear errno so that we can be sure that a nonzero value is caused by this function.
   errno = 0;
   
   // Allocate a call stack.
   int iCallStackCapacity = COMPACT_PATH_CALL_STACK_UNIT;
   int iNumCallsInStack = 0;
   CompactPathSubproblemCall *pCallStackBase = (CompactPathSubproblemCall *)
      malloc(sizeof(CompactPathSubproblemCall) * COMPACT_PATH_CALL_STACK_UNIT);
   
   if (errno || pCallStackBase == NULL)
      {
      COMPACT_PATH_RETURN(FAILURE);
      }

   // Set up the first instance of the problem.
   CompactPathSubproblemCall current = 
      {
      pPointArray,
      pResultPointArray,
      iPointsInCurrentPath,
      1 // 1 means copy first point. The main subproblem needs its first point copied to the result.
      };
   
   // Add the first instance to the stack
   if (!CompactPathCallStackPush(&pCallStackBase, &iCallStackCapacity, &iNumCallsInStack, &current))
      {
      COMPACT_PATH_RETURN(FAILURE);
      }
   
   // As long as there are calls on the stack, pop one and process it.
   while (iNumCallsInStack > 0)
      {
      if (!CompactPathCallStackPop(pCallStackBase, &iNumCallsInStack, &current))
         {
         COMPACT_PATH_RETURN(FAILURE);
         }
      
      subproblemResultCode = CompactPathSubproblemSolver(current.pPointArray,
         current.iPointsInCurrentPath, current.pResultPointArray, &iPointsInResultPath,
         &iDivisionIndex, dEpsilon);

      if (subproblemResultCode == COMPACT_PATH_RESULT_CODE_DIVIDE)
         {
         if (iDivisionIndex <= 0 || iDivisionIndex >= current.iPointsInCurrentPath)
            {
            COMPACT_PATH_RETURN(FAILURE);
            }
         else
            {
            // Create two new subproblems and push them.
            // It's important that the way that results are copied is compatible with the order in
            // which the subproblem calls are pushed to the stack.

            secondSubproblem.pPointArray = current.pPointArray + iDivisionIndex;
            secondSubproblem.pResultPointArray = current.pResultPointArray + iDivisionIndex;
            secondSubproblem.iPointsInCurrentPath = current.iPointsInCurrentPath - iDivisionIndex;
            // Never copy the first point of a right-side subproblem. This shared point will
            // always be copied at the return of the left-side subproblem.
            secondSubproblem.iCopyFirstPoint = 0; // 0 means do not copy.
            if (!CompactPathCallStackPush(&pCallStackBase, &iCallStackCapacity,
                                          &iNumCallsInStack, &secondSubproblem))
               {
               COMPACT_PATH_RETURN(FAILURE);
               }
            
            firstSubproblem.pPointArray = current.pPointArray;
            firstSubproblem.pResultPointArray = current.pResultPointArray;
            firstSubproblem.iPointsInCurrentPath = iDivisionIndex + 1;
            // Only copy the first point for the left subproblem when the just-returned call
            // needs its first point copied.
            firstSubproblem.iCopyFirstPoint = current.iCopyFirstPoint;
            if (!CompactPathCallStackPush(&pCallStackBase, &iCallStackCapacity,
                                          &iNumCallsInStack, &firstSubproblem))
               {
               COMPACT_PATH_RETURN(FAILURE);
               }
            }
         }
      else if (subproblemResultCode == COMPACT_PATH_RESULT_CODE_LINEARIZE ||
               subproblemResultCode == COMPACT_PATH_RESULT_CODE_SOLVED)
         {
         // Copy the results to their final destination in the result array.
         if (!current.iCopyFirstPoint)
            {
            ++current.pResultPointArray;
            --iPointsInResultPath;
            }
         
         // There's a good chance that the memory regions will overlap at some point.
         memmove(pResultPointArray + iNumSolvedPoints, current.pResultPointArray,
                 sizeof(DVector2D) * iPointsInResultPath);
         
         iNumSolvedPoints += iPointsInResultPath;
         }
      else
         {
         // Bad result code.
         COMPACT_PATH_RETURN(FAILURE);
         }
      }

   *piPointsInResultPath = iNumSolvedPoints;
   
   COMPACT_PATH_RETURN(SUCCESS);
   }

// If the result is COMPACT_PATH_RESULT_CODE_DIVIDE, it means that the algorithm needs to divide
// the problem into two smaller subproblems. In this case, divisionIndex is set to the
// index of the point that should be the end point of the first subproblem and the start point
// of the second subproblem. pointsInCurrentPath is not set, because its value is not yet known.
// If the result is COMPACT_PATH_RESULT_CODE_LINEARIZE, it means that all the intermediate points in
// the subproblem were removed. In this case, divisionIndex is not set.
// If the result is COMPACT_PATH_RESULT_CODE_SOLVED, it means that the algorithm does not need to
// do any further work on the subproblem, because it is already solved. In this case, divisionIndex
// is not set.
CompactPathResultCode CompactPathSubproblemSolver(DVector2D *pPointArray, int iPointsInCurrentPath,
      DVector2D *pResultPointArray, int *piPointsInResultPath, int *piDivisionIndex,
      double dEpsilon)
   {
   double dSquareSegLen, dDX, dDY, dArea, dSquareDeviation, dMaxSquareDeviationInThisSegment,
      ax, ay, bx, by, cx, cy;
   int i, iMaxPointIndex;
   
   // If there are fewer than three points provided, the problem is solved already.
   if (iPointsInCurrentPath < 3)
      {
      // Just copy pointArray into resultPointArray.
      if (iPointsInCurrentPath > 0)
         {
            memcpy(pResultPointArray, pPointArray, sizeof(DVector2D) * iPointsInCurrentPath);
         }
      *piPointsInResultPath = iPointsInCurrentPath;
      return COMPACT_PATH_RESULT_CODE_SOLVED;
      }
   
   dMaxDeviationInThisSegment = 0.0;
   ax = pPointArray[0].dX;
   ay = pPointArray[0].dY;
   cx = pPointArray[iPointsInCurrentPath-1].dX;
   cy = pPointArray[iPointsInCurrentPath-1].dY;
   dDX = cx - ax;
   dDY = cy - ay;
   dSquareSegLen = dDX * dDX + dDY * dDY;
   
   for (i = 1; i < iPointsInCurrentPath - 1; ++i)
      {
      bx = pPointArray[i].dX;
      by = pPointArray[i].dY;
      // TODO: optionally use the endpoint distance if the triangle is skew.
      dArea = ax * (by - cy) + bx * (cy - ay) + cx * (ay - by);
      
      dSquareDeviation = dArea * dArea / dSquareSegLen;
      if (dSquareDeviation > dMaxSquareDeviationInThisSegment)
         {
         iMaxPointIndex = i;
         dMaxSquareDeviationInThisSegment = dDeviation;
         }
      }
   
   if (dMaxSquareDeviationInThisSegment < dEpsilon * dEpsilon)
      {
      // Linearize the points in the subproblem.
      // To do this, we just copy the first and last points to the result array.
      pResultPointArray[0] = pPointArray[0];
      pResultPointArray[1] = pPointArray[iPointsInCurrentPath - 1];
      *piPointsInResultPath = 2;
      return COMPACT_PATH_RESULT_CODE_LINEARIZE;
      }
   else
      {
      // Split the subproblem.
      *piDivisionIndex = iMaxPointIndex;
      return COMPACT_PATH_RESULT_CODE_DIVIDE;
      }
   }
   