/*
   PathCompacterRecursive.c
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

#include <string.h> // For memcpy

#define FAILURE 0
#define SUCCESS 1

// This keeps track of the location of the "compacter". This is just the spot in the result
// array where the next points to be solved should be placed.
// Only CompactPath and CompactPathRecursive should touch this variable.
static DVector2D *pCompacterLocation;
   
static void CompactPathRecursive(DVector2D *pPointArray, int iPointsInCurrentPath, double dEpsilon);

// This function iteratively simulates the recursive Ramer-Douglas-Peucker algorithm.
// https://en.wikipedia.org/wiki/Ramer%E2%80%93Douglas%E2%80%93Peucker_algorithm
// Please allocate the resultPointArray to be as large as the pointArray passed in.
// It would be a good idea to resize the allocated space for resultPointArray after this
// function returns using the value of pointsInResultPath.
// This algorithm works in-place. That is, you can use the same array for both pointArray
// and resultPointArray, keeping in mind that doing so will likely alter pointArray.
// Returns a true value (1) on successful completion, and returns a false value (0) otherwise.
// This function might blow up the stack and crash your program.
int CompactPath(DVector2D *pPointArray, int iPointsInCurrentPath,
                DVector2D *pResultPointArray, int *piPointsInResultPath, double dEpsilon)
   {
   int iFinalSuccessValue;

   // Check for invalid values.
   if (iPointsInCurrentPath < 0 || pPointArray == NULL || pResultPointArray == NULL ||
       piPointsInResultPath == NULL || dEpsilon < 0.0)
      {
      return FAILURE;
      }
      
   // Copy the first point into the result. This can be done because its final location is
   // known (it will still be the first point), and it will certainly be in the final array
   // (it can never be removed).
   // The compacter skips copying the first point of each subproblem because it is added as the
   // last point of the subproblem before it. Copying the very first point is necessary
   // because the leftmost subproblem has no prior subproblem.
   *pResultPointArray = *pPointArray;
   pCompacterLocation = pResultPointArray + 1;

   // Launch the first call of CompactPathRecursive.
   CompactPathRecursive(pPointArray, iPointsInCurrentPath, dEpsilon);

   // Calculate and return through pointer the number of points in the resulting path.
   *piPointsInResultPath = pCompacterLocation - pResultPointArray;

   return SUCCESS;
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
static void CompactPathRecursive(DVector2D *pPointArray, int iPointsInCurrentPath, double dEpsilon)
   {
   double dSquareSegLen, dDX, dDY, dArea, dSquareDeviation, dMaxSquareDeviationInThisSegment,
      ax, ay, bx, by, cx, cy;
   int i, iMaxPointIndex;
   
   // If there are fewer than three points provided, the problem is solved already.
   if (iPointsInCurrentPath < 3)
      {
      // Just copy pointArray into the compacter.
      if (iPointsInCurrentPath > 0)
         {
            memcpy(pCompacterLocation, pPointArray + 1,
                   sizeof(DVector2D) * (iPointsInCurrentPath - 1));
            pCompacterLocation += iPointsInCurrentPath - 1;
         }
         
      return;
      }
   
   dMaxSquareDeviationInThisSegment = 0.0;
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
         dMaxSquareDeviationInThisSegment = dSquareDeviation;
         }
      }
   
   if (dMaxSquareDeviationInThisSegment < dEpsilon * dEpsilon)
      {
      // Linearize the points in the subproblem.
      // To do this, we just copy the last point into the compacter.

      *pCompacterLocation = pPointArray[iPointsInCurrentPath - 1];
      ++pCompacterLocation;
      }
   else
      {
      // Split the subproblem. Make sure to keep this left-recursive.
      
      CompactPathRecursive(pPointArray, iMaxPointIndex + 1, dEpsilon);

      CompactPathRecursive(pPointArray + iMaxPointIndex, iPointsInCurrentPath - iMaxPointIndex,
                           dEpsilon);
      }
      
   return;
   }
