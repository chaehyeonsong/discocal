#include "CCircleGridFinder.hpp"


CircleGridFinder::CircleGridFinder(bool _isAsymmetricGrid){
  isAsymmetricGrid= _isAsymmetricGrid;
  squareSize = 1.0f;
  maxRectifiedDistance = squareSize/2.0f;
}

void CircleGridFinder::hierarchicalClustering(const std::vector<cv::Point2f> &points, const cv::Size &patternSz, std::vector<cv::Point2f> &patternPoints)
{
    int j, n = (int)points.size();
    size_t pn = static_cast<size_t>(patternSz.area());

    patternPoints.clear();
    if (pn >= points.size())
    {
        if (pn == points.size())
            patternPoints = points;
        return;
    }

    cv::Mat dists(n, n, CV_32FC1, cv::Scalar(0));
    cv::Mat distsMask(dists.size(), CV_8UC1, cv::Scalar(0));
    for(int i = 0; i < n; i++)
    {
        for(j = i+1; j < n; j++)
        {
            dists.at<float>(i, j) = (float)norm(points[i] - points[j]);
            distsMask.at<uchar>(i, j) = 255;
            //TODO: use symmetry
            distsMask.at<uchar>(j, i) = 255;//distsMask.at<uchar>(i, j);
            dists.at<float>(j, i) = dists.at<float>(i, j);
        }
    }

    std::vector<std::list<size_t> > clusters(points.size());
    for(size_t i=0; i<points.size(); i++)
    {
        clusters[i].push_back(i);
    }

    int patternClusterIdx = 0;
    while(clusters[patternClusterIdx].size() < pn)
    {
        cv::Point minLoc;
        minMaxLoc(dists, 0, 0, &minLoc, 0, distsMask);
        int minIdx = std::min(minLoc.x, minLoc.y);
        int maxIdx = std::max(minLoc.x, minLoc.y);

        distsMask.row(maxIdx).setTo(0);
        distsMask.col(maxIdx).setTo(0);
        cv::Mat tmpRow = dists.row(minIdx);
        cv::Mat tmpCol = dists.col(minIdx);
        cv::min(dists.row(minLoc.x), dists.row(minLoc.y), tmpRow);
        tmpRow = tmpRow.t();
        tmpRow.copyTo(tmpCol);
        clusters[minIdx].splice(clusters[minIdx].end(), clusters[maxIdx]);
        patternClusterIdx = minIdx;
    }

    //the largest cluster can have more than pn points -- we need to filter out such situations
    if(clusters[patternClusterIdx].size() != static_cast<size_t>(patternSz.area()))
    {
      return;
    }

    patternPoints.reserve(clusters[patternClusterIdx].size());
    for(std::list<size_t>::iterator it = clusters[patternClusterIdx].begin(); it != clusters[patternClusterIdx].end();++it)
    {
        patternPoints.push_back(points[*it]);
    }
}

void CircleGridFinder::findGrid(const std::vector<cv::Point2f> &points, cv::Size _patternSize, std::vector<cv::Point2f>& centers)
{
  patternSize = _patternSize;
  centers.clear();
  if(points.empty())
  {
    return;
  }

  std::vector<cv::Point2f> patternPoints;
  hierarchicalClustering(points, patternSize, patternPoints);
  if(patternPoints.empty())
  {
    return;
  }


  std::vector<cv::Point2f> hull2f;
  convexHull(patternPoints, hull2f, false);
  const size_t cornersCount = isAsymmetricGrid ? 6 : 4;
  if(hull2f.size() < cornersCount)
    return;

  std::vector<cv::Point2f> corners;
  findCorners(hull2f, corners);
  if(corners.size() != cornersCount)
    return;

  std::vector<cv::Point2f> outsideCorners, sortedCorners;
  if(isAsymmetricGrid)
  {
    findOutsideCorners(corners, outsideCorners);
    const size_t outsideCornersCount = 2;
    if(outsideCorners.size() != outsideCornersCount)
      return;
  }
  getSortedCorners(hull2f, patternPoints, corners, outsideCorners, sortedCorners);
  if(sortedCorners.size() != cornersCount)
    return;

  std::vector<cv::Point2f> rectifiedPatternPoints;
  rectifyPatternPoints(patternPoints, sortedCorners, rectifiedPatternPoints);
  if(patternPoints.size() != rectifiedPatternPoints.size())
    return;

  parsePatternPoints(patternPoints, rectifiedPatternPoints, centers);
}

void CircleGridFinder::findCorners(const std::vector<cv::Point2f> &hull2f, std::vector<cv::Point2f> &corners)
{
  //find angles (cosines) of vertices in convex hull
  std::vector<float> angles;
  for(size_t i=0; i<hull2f.size(); i++)
  {
    cv::Point2f vec1 = hull2f[(i+1) % hull2f.size()] - hull2f[i % hull2f.size()];
    cv::Point2f vec2 = hull2f[(i-1 + static_cast<int>(hull2f.size())) % hull2f.size()] - hull2f[i % hull2f.size()];
    float angle = (float)(vec1.ddot(vec2) / (norm(vec1) * norm(vec2)));
    angles.push_back(angle);
  }

  //sort angles by cosine
  //corners are the most sharp angles (6)
  cv::Mat anglesMat = cv::Mat(angles);
  cv::Mat sortedIndices;
  cv::sortIdx(anglesMat, sortedIndices, cv::SORT_EVERY_COLUMN + cv::SORT_DESCENDING);
  CV_Assert(sortedIndices.type() == CV_32SC1);
  CV_Assert(sortedIndices.cols == 1);
  const int cornersCount = isAsymmetricGrid ? 6 : 4;
  cv::Mat cornersIndices;
  cv::sort(sortedIndices.rowRange(0, cornersCount), cornersIndices, cv::SORT_EVERY_COLUMN + cv::SORT_ASCENDING);
  corners.clear();
  for(int i=0; i<cornersCount; i++)
  {
    corners.push_back(hull2f[cornersIndices.at<int>(i, 0)]);
  }
}

void CircleGridFinder::findOutsideCorners(const std::vector<cv::Point2f> &corners, std::vector<cv::Point2f> &outsideCorners)
{
  CV_Assert(!corners.empty());
  outsideCorners.clear();
  //find two pairs of the most nearest corners
  const size_t n = corners.size();

  std::vector<cv::Point2f> tangentVectors(n);
  for(size_t k=0; k < n; k++)
  {
    cv::Point2f diff = corners[(k + 1) % n] - corners[k];
    tangentVectors[k] = diff * (1.0f / norm(diff));
  }

  //compute angles between all sides
  cv::Mat cosAngles((int)n, (int)n, CV_32FC1, 0.0f);
  for(size_t i = 0; i < n; i++)
  {
    for(size_t j = i + 1; j < n; j++)
    {
      float val = fabs(tangentVectors[i].dot(tangentVectors[j]));
      cosAngles.at<float>((int)i, (int)j) = val;
      cosAngles.at<float>((int)j, (int)i) = val;
    }
  }

  //find two parallel sides to which outside corners belong
  cv::Point maxLoc;
  minMaxLoc(cosAngles, 0, 0, 0, &maxLoc);
  const int diffBetweenFalseLines = 3;
  if(abs(maxLoc.x - maxLoc.y) == diffBetweenFalseLines)
  {
    cosAngles.row(maxLoc.x).setTo(0.0f);
    cosAngles.col(maxLoc.x).setTo(0.0f);
    cosAngles.row(maxLoc.y).setTo(0.0f);
    cosAngles.col(maxLoc.y).setTo(0.0f);
    minMaxLoc(cosAngles, 0, 0, 0, &maxLoc);
  }

  int maxIdx = std::max(maxLoc.x, maxLoc.y);
  int minIdx = std::min(maxLoc.x, maxLoc.y);
  const int bigDiff = 4;
  if(maxIdx - minIdx == bigDiff)
  {
    minIdx += (int)n;
    std::swap(maxIdx, minIdx);
  }
  if(maxIdx - minIdx != (int)n - bigDiff)
  {
    return;
  }

  int outsidersSegmentIdx = (minIdx + maxIdx) / 2;

  outsideCorners.push_back(corners[outsidersSegmentIdx % n]);
  outsideCorners.push_back(corners[(outsidersSegmentIdx + 1) % n]);
}

namespace {
double pointLineDistance(const cv::Point2f &p, const cv::Vec4f &line)
{
  cv::Vec3f pa( line[0], line[1], 1 );
  cv::Vec3f pb( line[2], line[3], 1 );
  cv::Vec3f l = pa.cross(pb);
  return std::abs((p.x * l[0] + p.y * l[1] + l[2])) * 1.0 /
         std::sqrt(double(l[0] * l[0] + l[1] * l[1]));
}
}

void CircleGridFinder::getSortedCorners(const std::vector<cv::Point2f> &hull2f, const std::vector<cv::Point2f> &patternPoints, const std::vector<cv::Point2f> &corners, const std::vector<cv::Point2f> &outsideCorners, std::vector<cv::Point2f> &sortedCorners)
{
  cv::Point2f firstCorner;
  if(isAsymmetricGrid)
  {
    cv::Point2f center = std::accumulate(corners.begin(), corners.end(), cv::Point2f(0.0f, 0.0f));
    center *= 1.0 / corners.size();

    std::vector<cv::Point2f> centerToCorners;
    for(size_t i=0; i<outsideCorners.size(); i++)
    {
      centerToCorners.push_back(outsideCorners[i] - center);
    }

    //TODO: use CirclesGridFinder::getDirection
    float crossProduct = centerToCorners[0].x * centerToCorners[1].y - centerToCorners[0].y * centerToCorners[1].x;
    //y axis is inverted in computer vision so we check > 0
    bool isClockwise = crossProduct > 0;
    firstCorner  = isClockwise ? outsideCorners[1] : outsideCorners[0];
  }
  else
  {
    firstCorner = corners[0];
  }

  std::vector<cv::Point2f>::const_iterator firstCornerIterator = std::find(hull2f.begin(), hull2f.end(), firstCorner);
  sortedCorners.clear();
  for(std::vector<cv::Point2f>::const_iterator it = firstCornerIterator; it != hull2f.end();++it)
  {
    std::vector<cv::Point2f>::const_iterator itCorners = std::find(corners.begin(), corners.end(), *it);
    if(itCorners != corners.end())
    {
      sortedCorners.push_back(*it);
    }
  }
  for(std::vector<cv::Point2f>::const_iterator it = hull2f.begin(); it != firstCornerIterator;++it)
  {
    std::vector<cv::Point2f>::const_iterator itCorners = std::find(corners.begin(), corners.end(), *it);
    if(itCorners != corners.end())
    {
      sortedCorners.push_back(*it);
    }
  }
  if(!isAsymmetricGrid)
  {
    double dist01 = norm(sortedCorners[0] - sortedCorners[1]);
    double dist12 = norm(sortedCorners[1] - sortedCorners[2]);
    // Use half the average distance between circles on the shorter side as threshold for determining whether a point lies on an edge.
    double thresh = std::min(dist01, dist12) /  std::min(patternSize.width, patternSize.height) / 2;
    size_t circleCount01 = 0;
    size_t circleCount12 = 0;
    cv::Vec4f line01( sortedCorners[0].x, sortedCorners[0].y, sortedCorners[1].x, sortedCorners[1].y );
    cv::Vec4f line12( sortedCorners[1].x, sortedCorners[1].y, sortedCorners[2].x, sortedCorners[2].y );
    // Count the circles along both edges.
    for (size_t i = 0; i < patternPoints.size(); i++)
    {
      if (pointLineDistance(patternPoints[i], line01) < thresh)
        circleCount01++;
      if (pointLineDistance(patternPoints[i], line12) < thresh)
        circleCount12++;
    }

    // Ensure that the edge from sortedCorners[0] to sortedCorners[1] is the one with more circles (i.e. it is interpreted as the pattern's width).
    if ((circleCount01 > circleCount12 && patternSize.height > patternSize.width) || (circleCount01 < circleCount12 && patternSize.height < patternSize.width))
    {
      for(size_t i=0; i<sortedCorners.size()-1; i++)
      {
        sortedCorners[i] = sortedCorners[i+1];
      }
      sortedCorners[sortedCorners.size() - 1] = firstCorner;
    }
  }
}

void CircleGridFinder::rectifyPatternPoints(const std::vector<cv::Point2f> &patternPoints, const std::vector<cv::Point2f> &sortedCorners, std::vector<cv::Point2f> &rectifiedPatternPoints)
{
  //indices of corner points in pattern
  std::vector<cv::Point> trueIndices;
  trueIndices.emplace_back(0, 0);
  trueIndices.emplace_back(patternSize.width - 1, 0);
  if(isAsymmetricGrid)
  {
    trueIndices.emplace_back(patternSize.width - 1, 1);
    trueIndices.emplace_back(patternSize.width - 1, patternSize.height - 2);
  }
  trueIndices.emplace_back(patternSize.width - 1, patternSize.height - 1);
  trueIndices.emplace_back(0, patternSize.height - 1);

  std::vector<cv::Point2f> idealPoints;
  for(size_t idx=0; idx<trueIndices.size(); idx++)
  {
    int i = trueIndices[idx].y;
    int j = trueIndices[idx].x;
    if(isAsymmetricGrid)
    {
      idealPoints.emplace_back((2*j + i % 2)*squareSize, i*squareSize);
    }
    else
    {
      idealPoints.emplace_back(j*squareSize, i*squareSize);
    }
  }

  cv::Mat homography = findHomography(sortedCorners, idealPoints, 0);
  cv::Mat rectifiedPointsMat;
  transform(patternPoints, rectifiedPointsMat, homography);
  rectifiedPatternPoints.clear();
  convertPointsFromHomogeneous(rectifiedPointsMat, rectifiedPatternPoints);
}

void CircleGridFinder::parsePatternPoints(const std::vector<cv::Point2f> &patternPoints, const std::vector<cv::Point2f> &rectifiedPatternPoints, std::vector<cv::Point2f> &centers)
{
  cv::flann::LinearIndexParams flannIndexParams;
  cv::flann::Index flannIndex(cv::Mat(rectifiedPatternPoints).reshape(1), flannIndexParams);

  centers.clear();
  for( int i = 0; i < patternSize.height; i++ )
  {
    for( int j = 0; j < patternSize.width; j++ )
    {
      cv::Point2f idealPt;
      if(isAsymmetricGrid)
        idealPt = cv::Point2f((2*j + i % 2)*squareSize, i*squareSize);
      else
        idealPt = cv::Point2f(j*squareSize, i*squareSize);

      cv::Mat query(1, 2, CV_32F, &idealPt);
      const int knn = 1;
      int indicesbuf[knn] = {0};
      float distsbuf[knn] = {0.f};
      cv::Mat indices(1, knn, CV_32S, &indicesbuf);
      cv::Mat dists(1, knn, CV_32F, &distsbuf);
      flannIndex.knnSearch(query, indices, dists, knn, cv::flann::SearchParams());
      centers.push_back(patternPoints.at(indicesbuf[0]));

      if(distsbuf[0] > maxRectifiedDistance)
      {
        centers.clear();
        return;
      }
    }
  }
}