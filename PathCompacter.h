/*
   PathCompacter.h
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

// The following struct represents a double precision 2D point.
typedef struct DVector2D
   {
   double dX;
   double dY;
   } DVector2D;

// Use a typedef'd function pointer to refer to the metric calculation callbacks.
// These accept the start and end point of the path or subproblem, and the intermediate
// point in consideration. The segment length is passed in as well so that the callback does
// not have to recalculate it every time it gets called.
typedef double (*DeviationMetric)(DVector2D startOfSegment, DVector2D endOfSegment,
                                  DVector2D point, double dSegmentLength);

// This function iteratively simulates the recursive Ramer-Douglas-Peucker algorithm.
// https://en.wikipedia.org/wiki/Ramer%E2%80%93Douglas%E2%80%93Peucker_algorithm
// Please allocate the resultPointArray to be as large as the pointArray passed in.
// It would be a good idea to resize the allocated space for resultPointArray after this
// function returns using the value of pointsInResultPath.
// This algorithm works in-place. That is, you can use the same array for both pointArray
// and resultPointArray, keeping in mind that doing so will likely alter pointArray.
// Returns a true value (1) on successful completion, and returns a false value (0) otherwise.
// On failure, pointsInResultPath and the contents of resultPointArray are undefined.
extern int compactPath(DVector2D *pPointArray, int iPointsInCurrentPath,
                       DVector2D *pResultPointArray, int *piPointsInResultPath,
                       double dEpsilon, CompactPathDeviationMetric deviationMetric);

// Declare several metric function implementations.

extern DeviationMetric perpendicularOffsetDeviationMetric;
extern DeviationMetric shortestDistanceToSegmentDeviationMetric;
