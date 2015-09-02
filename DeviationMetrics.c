/*
   DistanceMetrics.c
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

// This one just returns the shortest distance to the infinite extension of the line segment.
static double perpendicularDistance(DVector2D start, DVector2D end, DVector2D mid,
                                    double dSquareSegmentLength)
   {
   double dArea;

   dArea = 0.5 * (start.dX * (mid.dY - end.dY) +
                  mid.dX * (end.dY - start.dY) +
                  end.dX * (start.dY - mid.dY));

   return dArea * dArea / dSquareSegmentLength;
   }
DeviationMetric perpendicularDistanceDeviationMetric = &perpendicularDistance;

static double shortestDistanceToSegment(DVector2D start, DVector2D end, DVector2D mid,
                                        double dSquareSegmentLength)
   {
   double dAX, dAY, dBX, dBY, dCX, dCY, dAdotB, dBdotC, dArea;

   // Start->End forms vector A.
   dAX = end.dX - start.dX;
   dAY = end.dY - start.dY;

   // Start->Mid forms vector B.
   dBX = mid.dX - start.dX;
   dBY = mid.dY - start.dY;

   // End->Mid forms vector C;
   dCX = mid.dX - end.dX;
   dCY = mid.dY - end.dY;

   // Find the dot product of A and B.
   dAdotB = dAX * dBX + dAY * dBY;

   // Find the dot product of B and C;
   dBdotC = dBX * dCX + dBY * dCY;

   // If the signs are different, the closest point on the segment is not an endpoint.
   if (dAdotB > 0.0 && dBdotC < 0.0)
      {
      dArea = 0.5 * (start.dX * (mid.dY - end.dY) +
                     mid.dX * (end.dY - start.dY) +
                     end.dX * (start.dY - mid.dY));

      return dArea * dArea / dSquareSegmentLength;
      }

   // Otherwise, figure out which endpoint it is closer to.
   else
      {
      if (dAdotB < 0.0 && dBdotC < 0.0)
         {
         // It is closer to the start point.
         return dAX * dAX + dAY * dAY;
         }
      else
         {
         // It is closer to the end point.
         return dCX * dCX + dCY * dCY;
         }
      }
   }
DeviationMetric shortestDistanceToSegmentDeviationMetric = &shortestDistanceToSegment;
