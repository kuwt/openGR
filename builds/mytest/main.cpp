#include "super4pcs/algorithms/super4pcs.h"
#include "super4pcs/io/io.h"
#include "super4pcs/utils/geometry.h"

#include <Eigen/Dense>
#include "pointcloudFileIO.h"
#include "BasicVisualizer.h"

const std::string path1 = "data\\data_L.obj";    //data , building KD tree
const std::string path2 = "data\\model_L2.obj";  //Reference, for LCP reference

const float overlapFactor = 0.6;   // determine number of trials. smaller number leads to more trials and longer computing time
const float delta = 0.0005;		// determine when a point is a valid match. Affect LCP score.
const int sample_size = 500;   // determine number of points for matching. Greater number leads to longer computing time per trial.

int main(int argc, char **argv) {
  using namespace GlobalRegistration;
  using namespace std;

  vector<Point3D> set1, set2;
  vector<Eigen::Matrix2f> tex_coords1, tex_coords2;
  vector<typename Point3D::VectorType> normals1, normals2;
  vector<tripple> tris1, tris2;
  vector<std::string> mtls1, mtls2;

  IOManager iomanager;

  // dummy call, to test symbols accessibility
  iomanager.ReadObject(path1.c_str(), set1, tex_coords1, normals1, tris1, mtls1);
  iomanager.ReadObject(path2.c_str(), set2, tex_coords2, normals2, tris2, mtls2);


  // check availability of the Utils functions
  if (tris1.size() == 0)
    Utils::CleanInvalidNormals(set1, normals1);

  // Our matcher.
  Match4PCSOptions options;

  // Set parameters.
  Match4PCSBase::MatrixType mat;
  options.configureOverlap(overlapFactor);
  options.delta = delta;
  options.sample_size = sample_size;
  typename Point3D::Scalar score = 0;

  constexpr Utils::LogLevel loglvl = Utils::Verbose;
  using TrVisitorType = Match4PCSBase::DummyTransformVisitor;
  using SamplerType   = Match4PCSBase::DefaultSampler;
  Utils::Logger logger(loglvl);

  MatchSuper4PCS matcher(options, logger);
  score = matcher.ComputeTransformation<SamplerType,TrVisitorType>(set1, &set2, mat);

  logger.Log<Utils::Verbose>( "Score: ", score );

  iomanager.WriteMatrix("output.map", mat.cast<double>(), IOManager::POLYWORKS);

  cv::Mat obj1,obj2;
  {
	  pointcloudFileIO::LoadPointCloudFromObj(path1, obj1);
	  pointcloudFileIO::LoadPointCloudFromObj(path2, obj2);

	  BasicVisualizer::BasicVisualizer vis;

	  cv::Mat PointCloud;
	  vis.addPointCloud(obj2, "pointcloudTarget");
	  vis.addText("pointcloudTarget", 0, 12, 12, 1, 1, 1, "pointcloudTarget_text");
	  vis.setPointCloudRenderingProperties(BasicVisualizer::COLOR,
		  1.0,
		  1.0,
		  1.0,
		  "pointcloudTarget");

	  vis.addPointCloud(obj1, "pointcloudSrc");
	  vis.addText("pointcloudSrc", 0, 0, 12, 1, 1, 1, "pointcloudSrc_text");
	  vis.setPointCloudRenderingProperties(BasicVisualizer::COLOR,
		  0,
		  1.0,
		  0,
		  "pointcloudSrc");
	  cv::Mat TransM = cv::Mat(cv::Size(4,4),CV_32FC1);
	  TransM.at<float>(0, 0) = mat(0,0); TransM.at<float>(0, 1) = mat(0, 1); TransM.at<float>(0, 2) = mat(0, 2); TransM.at<float>(0, 3) = mat(0, 3);
	  TransM.at<float>(1, 0) = mat(1,0); TransM.at<float>(1, 1) = mat(1, 1); TransM.at<float>(1, 2) = mat(1, 2); TransM.at<float>(1, 3) = mat(1, 3);
	  TransM.at<float>(2, 0) = mat(2,0); TransM.at<float>(2, 1) = mat(2, 1); TransM.at<float>(2, 2) = mat(2, 2); TransM.at<float>(2, 3) = mat(2, 3);
	  TransM.at<float>(3, 0) = mat(3,0); TransM.at<float>(3, 1) = mat(3, 1); TransM.at<float>(3, 2) = mat(3, 2); TransM.at<float>(3, 3) = mat(3, 3);
	  vis.TransformObject(TransM, "pointcloudTarget");
	  vis.setBackgroundColor(0, 0, 0);
	  vis.Render();
	  vis.clearVisualizer();
  }
  return 0;
}

