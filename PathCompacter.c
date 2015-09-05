/*
   PathCompacter.c
   08/29/2015
   Authors: Michael Casebolt, Brett Casebolt
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

// This file uses lines up to 100 characters long. If this 100 character line fits then you're good.

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
   int uPointsInCurrentPath;
   } CompactPathSubproblemCall;
   
typedef enum CompactPathResultCode
   {
   COMPACT_PATH_RESULT_CODE_DIVIDE,
   COMPACT_PATH_RESULT_CODE_LINEARIZE,
   COMPACT_PATH_RESULT_CODE_SOLVED
   } CompactPathResultCode;

#define FAILURE 0
#define SUCCESS 1
   
static CompactPathResultCode compactPathSubproblemSolver(DVector2D *pPointArray,
      unsigned int uPointsInCurrentPath, DVector2D *pResultPointArray,
      unsigneed int *puPointsInResultPath, int *piDivisionIndex, double dEpsilon,
      DeviationMetric deviationMetric);

// The call stack will start able to hold this many calls and grow by this amount whenever it needs
// to grow in size.
#define COMPACT_PATH_CALL_STACK_UNIT 2048

// callStackBase is a double pointer because realloc might move the base pointer.
static int compactPathCallStackPush(CompactPathSubproblemCall **ppCallStackBase,
                                    int *piCallStackCapacity, int *piNumCallsInStack,
                                    CompactPathSubproblemCall *pCall)
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
         return FAILURE;
         }
      }
      
      // Add the new call
      (*ppCallStackBase)[*piNumCallsInStack] = *pCall;
      ++(*piNumCallsInStack);
      
      return SUCCESS;
   }

static int compactPathCallStackPop(CompactPathSubproblemCall *pCallStackBase,
                                   int *piNumCallsInStack, CompactPathSubproblemCall *pPoppedCall)
   {
   // Check if there is a call to pop.
   if (*piNumCallsInStack > 0)
      {
      --(*piNumCallsInStack);
      *pPoppedCall = pCallStackBase[*piNumCallsInStack];
      return SUCCESS;
      }
   else
      {
      // There are no calls to pop.
      return FAILURE;
      }
   }

// This is the cleanup macro for the CompactPath function.
// Free the only allocated memory and return the parameter
// as the return value.
#define COMPACT_PATH_RETURN(iReturnValue)\
   {\
   free(pCallStackBase);\
   return iReturnValue;\
   }
   
// This function iteratively simulates the recursive Ramer-Douglas-Peucker algorithm.
// https://en.wikipedia.org/wiki/Ramer%E2%80%93Douglas%E2%80%93Peucker_algorithm
// Please allocate the resultPointArray to be as large as the pointArray passed in.
// It would be a good idea to resize the allocated space for resultPointArray after this
// function returns using the value of pointsInResultPath.
// This algorithm works in-place. That is, you can use the same array for both pointArray
// and resultPointArray, keeping in mind that doing so will likely alter pointArray.
// Returns a true value (1) on successful completion, and returns a false value (0) otherwise.
// On failure, pointsInResultPath and the contents of resultPointArray are undefined.
int compactPath(DVector2D *pPointArray, unsigned int uPointsInCurrentPath,
                DVector2D *pResultPointArray, unsigned int *puPointsInResultPath,
                double dEpsilon, DeviationMetric deviationMetric)
   {
   CompactPathSubproblemCall current, firstSubproblem, secondSubproblem;
   int iDivisionIndex; // Where should we split the problem into subproblems?
   CompactPathResultCode subproblemResultCode; // The status of the most recent subproblem call
   int iNumSolvedPoints; // Keep track of how much of the result array is solved and in place.
   int uPointsInResultPath; // Number of valid points in the result array after a subproblem call
   int iCallStackCapacity; // How many calls can the call stack hold right now?
   int iNumCallsInStack; // How many calls are in the call stack right now?
   
   // Clear errno so that we can be sure that a nonzero value is caused by this function.
   errno = 0;

   // Copy the first point into the result. This can be done because its final location is
   // known (it will still be the first point), and it will certainly be in the final array
   // (it can never be removed).
   // The compacter skips copying the first point of each subproblem because it is added as the
   // last point of the subproblem before it. Copying the very first point is necessary
   // because the leftmost subproblem has no prior subproblem.
   *pResultPointArray = *pPointArray;
   iNumSolvedPoints = 1;
   
   // Allocate a call stack.
   iCallStackCapacity = COMPACT_PATH_CALL_STACK_UNIT;
   iNumCallsInStack = 0;
   CompactPathSubproblemCall *pCallStackBase = (CompactPathSubproblemCall *)
      malloc(sizeof(CompactPathSubproblemCall) * COMPACT_PATH_CALL_STACK_UNIT);
   
   if (errno || pCallStackBase == NULL)
      {
      COMPACT_PATH_RETURN(FAILURE);
      }

   // Set up the first instance of the problem, representing the whole problem.
   current.pPointArray = pPointArray; 
   current.pResultPointArray = pResultPointArray;
   current.uPointsInCurrentPath = uPointsInCurrentPath;
   
   // Add the first instance to the stack
   if (!compactPathCallStackPush(&pCallStackBase, &iCallStackCapacity, &iNumCallsInStack, &current))
      {
      COMPACT_PATH_RETURN(FAILURE);
      }
   
   // As long as there are calls on the stack, pop one and process it.
   while (iNumCallsInStack > 0)
      {
      if (!compactPathCallStackPop(pCallStackBase, &iNumCallsInStack, &current))
         {
         COMPACT_PATH_RETURN(FAILURE);
         }
      
      subproblemResultCode = compactPathSubproblemSolver(current.pPointArray,
         current.uPointsInCurrentPath, current.pResultPointArray, &uPointsInResultPath,
         &iDivisionIndex, dEpsilon, deviationMetric);

      if (subproblemResultCode == COMPACT_PATH_RESULT_CODE_DIVIDE)
         {
         if (iDivisionIndex <= 0 || iDivisionIndex >= current.uPointsInCurrentPath)
            {
            COMPACT_PATH_RETURN(FAILURE);
            }
         else
            {
            // Create two new subproblems and push them.
            // It's important that the way that results are copied is compatible with the order in
            // which the subproblem calls are pushed to the stack.
            // This function is left-recursive. It performs left-side subproblems before
            // right-side subproblems. This means that the left-side subproblem needs to be
            // pushed to the stack last, so that it is popped back out first.

            secondSubproblem.pPointArray = current.pPointArray + iDivisionIndex;
            secondSubproblem.pResultPointArray = current.pResultPointArray + iDivisionIndex;
            secondSubproblem.uPointsInCurrentPath = current.uPointsInCurrentPath - iDivisionIndex;
            if (!compactPathCallStackPush(&pCallStackBase, &iCallStackCapacity,
                                          &iNumCallsInStack, &secondSubproblem))
               {
               COMPACT_PATH_RETURN(FAILURE);
               }
            
            firstSubproblem.pPointArray = current.pPointArray;
            firstSubproblem.pResultPointArray = current.pResultPointArray;
            firstSubproblem.uPointsInCurrentPath = iDivisionIndex + 1;
            if (!compactPathCallStackPush(&pCallStackBase, &iCallStackCapacity,
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

         // Always skip copying the first point.
         ++current.pResultPointArray;
         --uPointsInResultPath;
         
         // There's a good chance that the memory regions will overlap at some point.
         memmove(pResultPointArray + iNumSolvedPoints, current.pResultPointArray,
                 sizeof(DVector2D) * uPointsInResultPath);
         
         iNumSolvedPoints += uPointsInResultPath;
         }
      else
         {
         // Bad result code.
         COMPACT_PATH_RETURN(FAILURE);
         }
      }

   *puPointsInResultPath = iNumSolvedPoints;
   
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
static CompactPathResultCode compactPathSubproblemSolver(DVector2D *pPointArray,
      int uPointsInCurrentPath, DVector2D *pResultPointArray,
      int *puPointsInResultPath, int *piDivisionIndex, double dEpsilon,
      DeviationMetric deviationMetric)
   {
   double dSquareSegLen, dSquareDeviation, dMaxSquareDeviationInThisSegment, dDX, dDY;
   int i, iMaxPointIndex;
   
   // If there are fewer than three points provided, the problem is solved already.
   if (uPointsInCurrentPath < 3)
      {
      // Just copy pointArray into resultPointArray.
      if (uPointsInCurrentPath > 0)
         {
            memcpy(pResultPointArray, pPointArray, sizeof(DVector2D) * uPointsInCurrentPath);
         }
      *puPointsInResultPath = uPointsInCurrentPath;
      return COMPACT_PATH_RESULT_CODE_SOLVED;
      }
   
   dMaxSquareDeviationInThisSegment = 0.0;

   dDX = pPointArray[uPointsInCurrentPath - 1].dX - pPointArray[0].dX;
   dDY = pPointArray[uPointsInCurrentPath - 1].dY - pPointArray[0].dY;
   dSquareSegLen = dDX * dDX + dDY * dDY;
   
   for (i = 1; i < uPointsInCurrentPath - 1; ++i)
      {
      dSquareDeviation = deviationMetric(pPointArray[0],
         pPointArray[uPointsInCurrentPath - 1], pPointArray[i], dSquareSegLen);

      if (dSquareDeviation > dMaxSquareDeviationInThisSegment)
         {
         iMaxPointIndex = i;
         dMaxSquareDeviationInThisSegment = dSquareDeviation;
         }
      }
   
   if (dMaxSquareDeviationInThisSegment < dEpsilon * dEpsilon)
      {
      // Linearize the points in the subproblem.
      // To do this, we just copy the first and last points to the result array.
      pResultPointArray[0] = pPointArray[0];
      pResultPointArray[1] = pPointArray[uPointsInCurrentPath - 1];
      *puPointsInResultPath = 2;
      return COMPACT_PATH_RESULT_CODE_LINEARIZE;
      }
   else
      {
      // Split the subproblem.
      *piDivisionIndex = iMaxPointIndex;
      return COMPACT_PATH_RESULT_CODE_DIVIDE;
      }
   }
