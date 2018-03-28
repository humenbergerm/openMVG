// This file was taken from OpenMVG and modified by Martin Humenberger, Naver Labs Europe

#include "openMVG/cameras/cameras.hpp"
#include "openMVG/cameras/Camera_IO.hpp"
#include "openMVG/exif/exif_IO_EasyExif.hpp"
#include "openMVG/exif/sensor_width_database/ParseDatabase.hpp"
#include "openMVG/geodesy/geodesy.hpp"
#include "openMVG/image/image_io.hpp"
#include "openMVG/numeric/eigen_alias_definition.hpp"
#include "openMVG/sfm/sfm_data.hpp"
#include "openMVG/sfm/sfm_data_io.hpp"
#include "openMVG/sfm/sfm_data_utils.hpp"
#include "openMVG/sfm/sfm_view.hpp"
#include "openMVG/sfm/sfm_view_priors.hpp"
#include "openMVG/types.hpp"

#include "third_party/cmdLine/cmdLine.h"
#include "third_party/progress/progress_display.hpp"
#include "third_party/stlplus3/filesystemSimplified/file_system.hpp"

#include <fstream>
#include <memory>
#include <string>
#include <utility>

using namespace openMVG;
using namespace openMVG::cameras;
using namespace openMVG::exif;
using namespace openMVG::geodesy;
using namespace openMVG::image;
using namespace openMVG::sfm;

// Naive function for finding the biggest common root dir from two paths
std::string FindCommonRootDir(const std::string & dir1, const std::string & dir2)
{
  int i = 0;
  for (; i != std::min(dir1.size(), dir2.size()); i++)
  {
    if (dir1[i] != dir2[i]) break;
  }
  if (stlplus::is_folder(dir1.substr(0,i)))
    return dir1.substr(0,i);
  else
    return stlplus::folder_part(dir1.substr(0,i));
}

/// Check that Kmatrix is a string like "f;0;ppx;0;f;ppy;0;0;1"
/// With f,ppx,ppy as valid numerical value
bool checkIntrinsicStringValidity(const std::string &Kmatrix, double &focal, double &ppx, double &ppy)
{
  std::vector<std::string> vec_str;
  stl::split(Kmatrix, ';', vec_str);
  if (vec_str.size() != 9)
  {
    std::cerr << "\n Missing ';' character" << std::endl;
    return false;
  }
  // Check that all K matrix value are valid numbers
  for (size_t i = 0; i < vec_str.size(); ++i)
  {
    double readvalue = 0.0;
    std::stringstream ss;
    ss.str(vec_str[i]);
    if (!(ss >> readvalue))
    {
      std::cerr << "\n Used an invalid not a number character" << std::endl;
      return false;
    }
    if (i == 0)
    {
      focal = readvalue;
    }
    if (i == 2)
    {
      ppx = readvalue;
    }
    if (i == 5)
    {
      ppy = readvalue;
    }
  }
  return true;
}

std::pair<bool, Vec3> checkGPS(const std::string &filename, const int &GPS_to_XYZ_method = 0)
{
  std::pair<bool, Vec3> val(false, Vec3::Zero());
  std::unique_ptr<Exif_IO> exifReader(new Exif_IO_EasyExif);
  if (exifReader)
  {
    // Try to parse EXIF metada & check existence of EXIF data
    if (exifReader->open(filename) && exifReader->doesHaveExifInfo())
    {
      // Check existence of GPS coordinates
      double latitude, longitude, altitude;
      if (exifReader->GPSLatitude(&latitude) && exifReader->GPSLongitude(&longitude) &&
          exifReader->GPSAltitude(&altitude))
      {
        // Add ECEF or UTM XYZ position to the GPS position array
        val.first = true;
        switch (GPS_to_XYZ_method)
        {
          case 1:
            val.second = lla_to_utm(latitude, longitude, altitude);
            break;
          case 0:
          default:
            val.second = lla_to_ecef(latitude, longitude, altitude);
            break;
        }
      }
    }
  }
  return val;
}

/// Check string of prior weights
std::pair<bool, Vec3> checkPriorWeightsString(const std::string &sWeights)
{
  std::pair<bool, Vec3> val(true, Vec3::Zero());
  std::vector<std::string> vec_str;
  stl::split(sWeights, ';', vec_str);
  if (vec_str.size() != 3)
  {
    std::cerr << "\n Missing ';' character in prior weights" << std::endl;
    val.first = false;
  }
  // Check that all weight values are valid numbers
  for (size_t i = 0; i < vec_str.size(); ++i)
  {
    double readvalue = 0.0;
    std::stringstream ss;
    ss.str(vec_str[i]);
    if (!(ss >> readvalue))
    {
      std::cerr << "\n Used an invalid not a number character in local frame origin" << std::endl;
      val.first = false;
    }
    val.second[i] = readvalue;
  }
  return val;
}

/// Check that T is a string like "qw;qx;qy;qz"
/// with qw,qx,qy,qz as valid numerical values
/// and convert it to qT and tT.
/*bool checkTransformStringValidity(const std::string & T, Eigen::Quaterniond & qT, Eigen::Vector3d & tT)
{
  std::vector<std::string> vec_str;
  stl::split(T, ';', vec_str);
  if (vec_str.size() != 7)  {
    std::cerr << "\n Missing ';' character or wrong format" << std::endl;
    return false;
  }
  double tmp[7];
  // Check that all K matrix value are valid numbers
  for (size_t i = 0; i < vec_str.size(); ++i)
  {
    double readvalue = 0.0;
    std::stringstream ss;
    ss.str(vec_str[i]);
    if (! (ss >> readvalue) )  {
      std::cerr << "\n Used an invalid not a number character" << std::endl;
      return false;
    }
    tmp[i] = readvalue;
  }
  tT.x() = tmp[0];
  tT.y() = tmp[1];
  tT.z() = tmp[2];
  qT.w() = tmp[3];
  qT.x() = tmp[4];
  qT.y() = tmp[5];
  qT.z() = tmp[6];
  return true;
}*/

/**
* @brief Transform pose p
* @param p pose to transform
* @param q quaternion (rotation part of transform)
* @param t 3d vector (translation part of transform)
* @return new_pose.center = q*p.center + t
*         new_pose.rotation = q*p.rotation
*/
/*geometry::Pose3 transformPose3(const geometry::Pose3 & p, const Eigen::Quaterniond & q, const Eigen::Vector3d & t)
{
  return {q.toRotationMatrix() * p.rotation(),
          q.toRotationMatrix() * p.center() + t};
}*/

/// load poses in NLK format
std::vector<geometry::Pose3> loadPosesNLK(const std::string &filename)
{
  std::ifstream ifs(filename.c_str(), std::ifstream::in);

  if (!ifs.is_open())
  {
    std::cerr << "Error: file " + filename + " does not exist!" << std::endl;
    exit(1);
  }

  std::vector<geometry::Pose3> vec_poses;

  while (!ifs.eof())
  {
    char imgname[1024];
    double cx = 0.f, cy = 0.f, cz = 0.f;
    double qx, qy, qz, qw;

    std::string line("");
    getline(ifs, line);

    if (line != "")
    {
      sscanf(line.c_str(), "%s %lf %lf %lf %lf %lf %lf %lf", imgname, &cx, &cy, &cz, &qw, &qx, &qy, &qz);

      // we don't need the image name here
      std::string str = imgname;
      str += ".jpg";

      // needed to rotate the NLK orientations to the notation used in openMVG (z axis points into camera viewing direction)
      Quaternion qtr(0.5, 0.5, -0.5, 0.5); // Euler angles: order x,y,z -> 90deg,0,90deg

      Quaternion qt(qw, qx, qy, qz);
      Vec3 center(cx, cy, cz);

      Mat3 Rw = qt.toRotationMatrix();
      Mat3 Rwt = Rw.transpose();  // needed to be inverted (transposed) because NLK format defines the camera pose in the world frame and openMVG needs the rotation of the extrinsic camera parameters, which is the inverse of the rotation part of the pose
      Mat3 R = qtr.toRotationMatrix() * Rwt;

      geometry::Pose3 pose(R, center);
      vec_poses.push_back(pose);
    }
  }

  ifs.close();

  return vec_poses;
}

double closest_elem_in_map(std::map<double, std::pair<Mat3, Vec3>> const& map, double value)
{
  auto it = map.lower_bound(value);
  if (it == map.end())
  {
    it = map.upper_bound(value);
  }

  return it->first;
}

/// load poses in TUM format
std::vector<geometry::Pose3> loadPosesTUM(const std::string &filename, const std::vector<std::string> &vec_image)
{
  std::ifstream ifs(filename.c_str(), std::ifstream::in);

  if (!ifs.is_open())
  {
    std::cerr << "Error: file " + filename + " does not exist!" << std::endl;
    exit(1);
  }

  std::map<double, std::pair<Mat3, Vec3>> map_time_pose;
  std::vector<geometry::Pose3> vec_poses;

  while (!ifs.eof())
  {
    double cx = 0.f, cy = 0.f, cz = 0.f;
    double qx = 0.f, qy = 0.f, qz = 0.f, qw = 1.f;
    double timestamp = 0.f;

    std::string line("");
    getline(ifs, line);

    if (line != "" && line[0] != '#')
    {
      sscanf(line.c_str(), "%lf %lf %lf %lf %lf %lf %lf %lf", &timestamp, &cx, &cy, &cz, &qx, &qy, &qz, &qw);

      // needed to rotate the NLK orientations to the notation used in openMVG (z axis points into camera viewing direction)
      //Quaternion qtr(0.5, 0.5, -0.5, 0.5); // Euler angles: order x,y,z -> 90deg,0,90deg

      Quaternion qt(qw, qx, qy, qz);
      Vec3 center(cx, cy, cz);

      Mat3 Rw = qt.toRotationMatrix();
      Mat3 R = Rw.transpose();  // needed to be inverted (transposed) because NLK format defines the camera pose in the world frame and openMVG needs the rotation of the extrinsic camera parameters, which is the inverse of the rotation part of the pose
      //Mat3 R = qtr.toRotationMatrix() * Rwt;

      map_time_pose.insert({timestamp, {R, center}});
    }
  }

  ifs.close();

  for (auto & file_name : vec_image)
  {
    std::string str = stlplus::basename_part(file_name);
    double timestamp_file = atof(str.c_str());
    double key = closest_elem_in_map(map_time_pose, timestamp_file);
    std::pair<Mat3, Vec3> pose_pair = map_time_pose[key];
    geometry::Pose3 pose(pose_pair.first, pose_pair.second);
    vec_poses.push_back(pose);
  }

  return vec_poses;
}

static bool read_openMVG_Camera(const std::string & camName, cameras::PinholeCamera & cam)
{
  std::vector<double> val;
  if (stlplus::extension_part(camName) == "bin")
  {
    std::ifstream in(camName.c_str(), std::ios::in|std::ios::binary);
    if (!in.is_open())  {
      std::cerr << "Error: failed to open file '" << camName << "' for reading" << std::endl;
      return false;
    }
    val.resize(12);
    in.read((char*)&val[0],(std::streamsize)12*sizeof(double));
    if (in.fail())  {
      val.clear();
      return false;
    }
  }
  else
  {
    std::ifstream ifs;
    ifs.open( camName.c_str(), std::ifstream::in);
    if (!ifs.is_open()) {
      std::cerr << "Error: failed to open file '" << camName << "' for reading" << std::endl;
      return false;
    }
    while (ifs.good())
    {
      double valT;
      ifs >> valT;
      if (!ifs.fail())
        val.push_back(valT);
    }
  }

  //Update the camera from file value
  Mat34 P;
  if (stlplus::extension_part(camName) == "bin")
  {
    P << val[0], val[3], val[6], val[9],
      val[1], val[4], val[7], val[10],
      val[2], val[5], val[8], val[11];
  }
  else
  {
    P << val[0], val[1], val[2], val[3],
      val[4], val[5], val[6], val[7],
      val[8], val[9], val[10], val[11];
  }
  Mat3 K, R;
  Vec3 t;
  KRt_From_P(P, &K, &R, &t);
  cam = cameras::PinholeCamera(K, R, t);
  return true;
}

static bool read_Strecha_Camera(const std::string & camName, cameras::PinholeCamera & cam)
{
  std::ifstream ifs;
  ifs.open( camName.c_str(), std::ifstream::in);
  if (!ifs.is_open()) {
    std::cerr << "Error: failed to open file '" << camName << "' for reading" << std::endl;
    return false;
  }
  std::vector<double> val;
  while (ifs.good() && !ifs.eof())
  {
    double valT;
    ifs >> valT;
    if (!ifs.fail())
      val.push_back(valT);
  }

  if (val.size() == 3*3 +3 +3*3 +3 + 3 || val.size() == 26) //Strecha cam
  {
    Mat3 K, R;
    K << val[0], val[1], val[2],
      val[3], val[4], val[5],
      val[6], val[7], val[8];
    R << val[12], val[13], val[14],
      val[15], val[16], val[17],
      val[18], val[19], val[20];

    Vec3 C (val[21], val[22], val[23]);
    // Strecha model is P = K[R^T|-R^T t];
    // My model is P = K[R|t], t = - RC
    const Vec3 t (-R.transpose() * C);
    R.transposeInPlace();
    cam = cameras::PinholeCamera(K, R, t);
  }
  else
  {
    return false;
  }
  return true;
}

/**
@brief Reads a set of Pinhole Cameras and its poses from a ground truth dataset.
@param[in] functorPointer, to the function which can handle the trajectory format. Example: &read_openMVG_Camera
@param[in] sGTPath, the directory where the camera files are located.
@param[in] Suffix: use "bin" for openMVG or "png.camera" for Strechas data.
@param[out] vec_filenames: read cameras names
@param[out] map_Rt_gt: Map of poses
@param[out] map_camerasGT Map of PinholeCameras.
**/
bool readGt(
  bool (*fcnReadCamPtr)(const std::string &, cameras::PinholeCamera &), //pointer to the function reading a camera
  const std::string & sGTPath,
  const std::string & suffix,
  std::vector<std::string> & vec_filenames,
  std::map<std::string, std::pair<Mat3, Vec3>> & map_Rt_gt,
  std::map<size_t, cameras::PinholeCamera, std::less<size_t>,
    Eigen::aligned_allocator<std::pair<const size_t, cameras::PinholeCamera>>> & map_camerasGT)
{
  // IF GT_Folder exists, perform evaluation of the quality of rotation estimates
  if (!stlplus::is_folder(sGTPath)) {
    std::cout << std::endl << "There is not valid GT data to read from " << sGTPath << std::endl;
    return false;
  }
  else
  {
    std::cout << std::endl << "Read rotation and translation estimates" << std::endl;
    // Load GT
    std::map<std::string, Mat3, Eigen::aligned_allocator<Mat3>> map_R_gt;
    //Try to read .suffix camera (parse camera names)
    std::vector<std::string> vec_camfilenames =
      stlplus::folder_wildcard(sGTPath, "*." + suffix, false, true);
    std::sort(vec_camfilenames.begin(), vec_camfilenames.end());
    if (!vec_camfilenames.empty())
    {
      for (std::vector<std::string>::const_iterator iter = vec_camfilenames.begin();
           iter != vec_camfilenames.end(); ++iter) {
        cameras::PinholeCamera cam;
        if (fcnReadCamPtr(stlplus::create_filespec(sGTPath, *iter), cam))
        {
          vec_filenames.push_back(stlplus::create_filespec(sGTPath, *iter));
          map_Rt_gt.insert({stlplus::basename_part(*iter), {cam._R, cam._t}});
          map_camerasGT.insert({std::distance(vec_camfilenames.cbegin(), iter), cam});
        }
        else
          return false;
      }
    }
  }
  return true;
}

std::map<std::string, std::shared_ptr<openMVG::cameras::IntrinsicBase> >
parseCalibrationFile(const std::string &filename)
{
  std::map<std::string, std::shared_ptr<openMVG::cameras::IntrinsicBase> > intrinsic_list;
  if (!filename.empty())
  {
    std::ifstream file(filename.c_str());
    if (!file)
    {
      std::cerr << "Cannot open " << filename << std::endl;
      return intrinsic_list;
    }
    std::string line;
    while (file.good())
    {
      getline(file, line);
      if (!line.empty())
      {
        std::vector<std::string> values;
        stl::split(line, ' ', values);
        if (values.size() != 11 && values.size() != 12)
        {
          std::cerr << "Wrong camera parameters, expected 11 or 12 parameters but '" << line << "' was given.\n";
          return intrinsic_list;
        }
        float width = stof(values[1]);
        float height = stof(values[2]);
        float focal = (stof(values[3]) + stof(values[4])) / 2.0;
        float ppx = stof(values[5]);
        float ppy = stof(values[6]);
        float k1 = stof(values[7]);
        float k2 = stof(values[8]);
        float k3;
        if (values.size() == 12)
          k3 = stof(values[11]);
        else
          k3 = 0.0;
        float p1 = stof(values[9]);
        float p2 = stof(values[10]);
        intrinsic_list[values[0]] = std::make_shared<openMVG::cameras::Pinhole_Intrinsic_Brown_T2>(width, height, focal,
                                                                                                   ppx, ppy, k1, k2,
                                                                                                   k3, p1, p2);
        std::vector<double> camera_params = intrinsic_list[values[0]]->getParams();
        std::cout << "camera parameters (focal cx cy k1 k2 k3 p1 p2) for " << values[0] << ": ";
        for (double camera_param : camera_params)
          std::cout << camera_param << " ";
        std::cout << std::endl;
      }
    }
  }

  return intrinsic_list;
}

//
// Create the description of an input image dataset for OpenMVG toolsuite
// - Export a SfM_Data file with View & Intrinsic data
//
int main(int argc, char **argv)
{
  CmdLine cmd;

  std::string sImageDir, sfileDatabase = "", sOutputDir = "", sKmatrix, sGroundTruthPath = "",
    sSfmDataName = "sfm_data.json", sCalibrationFile = "";

  std::string sPriorWeights;
  std::pair<bool, Vec3> prior_w_info(false, Vec3(1.0, 1.0, 1.0));

  int i_User_camera_model = PINHOLE_CAMERA_RADIAL3;

  bool b_Group_camera_model = true;

  int i_GPS_XYZ_method = 0;

  double focal_pixels = -1.0;

  int i_gt_type = 1;

  cmd.add(make_option('i', sImageDir, "imageDirectory"));
  cmd.add(make_option('t', sGroundTruthPath, "gtPath"));
  cmd.add(make_option('s', i_gt_type, "gtType"));
  cmd.add(make_switch('T', "use_traj_prior_center"));
  cmd.add(make_switch('R', "use_traj_prior_rot"));
  cmd.add(make_option('d', sfileDatabase, "sensorWidthDatabase"));
  cmd.add(make_option('o', sOutputDir, "outputDirectory"));
  cmd.add(make_option('n', sSfmDataName, "sfmDataName"));
  cmd.add(make_option('f', focal_pixels, "focal"));
  cmd.add(make_option('k', sKmatrix, "intrinsics"));
  cmd.add(make_option('c', i_User_camera_model, "cameraModel"));
  cmd.add(make_option('g', b_Group_camera_model, "groupCameraModel"));
  cmd.add(make_switch('P', "use_pose_prior"));
  cmd.add(make_option('W', sPriorWeights, "priorWeights"));
  cmd.add(make_option('m', i_GPS_XYZ_method, "gps_to_xyz_method"));
  cmd.add(make_switch('G', "writeCameraFiles"));
  cmd.add(make_option('C', sCalibrationFile, "cameraCalibrationFile"));
  cmd.add(make_switch('A', "appendData"));
  cmd.add(make_switch('B', "parentDirAsRoot"));

  try
  {
    if (argc == 1)
    {
      throw std::string("Invalid command line parameter.");
    }
    cmd.process(argc, argv);
  } catch (const std::string &s)
  {
    std::cerr << "Usage: " << argv[0] << '\n'
              << "[-i|--imageDirectory]\n"
              << "[-o|--outputDirectory] sfm data and camera files will be written to this directory\n"
              << "[-t|--gtPath] path to ground truth data\n"
              << "[-s|--gtType] ground truth type \n"
              << "\t 1: (default) NLK trajectory file (format of poses in file: camname x y z qw qx qy qz)\n"
              << "\t\t gtPath needs to be the full path to the trajectory file\n"
              << "\t 2: Strecha's format\n"
              << "\t\t gtPath needs to be the full path to directory containing the camera files\n"
              << "\t 3: TUM trajectory file (format of poses in file: timestamp x y z qx qy qz qw)\n"
              << "[-T|--use_traj_prior_center] Use center of trajectory as pose prior\n"
              << "[-R|--use_traj_prior_rot] Use rotation of trajectory as pose prior\n"
              << "[-d|--sensorWidthDatabase]\n"
              << "[-o|--outputDirectory]\n"
              << "[-f|--focal] (pixels)\n"
              << "[-n|--sfmDataName] default is sfm_data.json\n"
              << "[-A|--appendData] append data to existing sfm_data file\n"
              << "[-B|--parentDirAsRoot] set parent directory of --imageDirectory as root in sfm_data file\n"
              << "[-k|--intrinsics] Kmatrix: \"f;0;ppx;0;f;ppy;0;0;1\"\n"
              << "[-C|--cameraCalibrationFile] format: camera_name image_width image_height fx fy cx cy k1 k2 p1 p2\n"
              << "[-c|--camera_model] Camera model type\n"
              << "\t 1: Pinhole\n"
              << "\t 2: Pinhole radial 1\n"
              << "\t 3: Pinhole radial 3 (default)\n"
              << "\t 4: Pinhole brown 2\n"
              << "\t 5: Pinhole with a simple Fish-eye distortion\n"
              << "\t 7: Spherical camera\n"
              << "[-g|--group_camera_model]\n"
              << "\t 0-> each view have it's own camera intrinsic parameters,\n"
              << "\t 1-> (default) view can share some camera intrinsic parameters\n" << "\n"
              << "[-P|--use_pose_prior] Use pose prior if GPS EXIF pose is available\n"
              << "[-W|--prior_weights] \"x;y;z;\" of weights for each dimension of the prior (default: 1.0)\n"
              << "[-m|--gps_to_xyz_method] XZY Coordinate system:\n" << "\t 0: ECEF (default)\n" << "\t 1: UTM\n"
              << "[-G|--writeCameraFiles] poses from trajectory files will be writen as camera files in outputDirectory\n"
              << std::endl;

    std::cerr << s << std::endl;
    return EXIT_FAILURE;
  }

  std::cout << " You called : " << std::endl << argv[0] << std::endl
            << "--imageDirectory " << sImageDir << std::endl
            << "--gtPath " << sGroundTruthPath << std::endl
            << "--gtType " << i_gt_type << std::endl
            << "--use_traj_prior_center " << cmd.used('T') << std::endl
            << "--use_traj_prior_rot " << cmd.used('R') << std::endl
            << "--sensorWidthDatabase " << sfileDatabase << std::endl
            << "--outputDirectory " << sOutputDir << std::endl
            << "--sfmDataName " << sSfmDataName << std::endl
            << "--appendData " << cmd.used('A') << std::endl
            << "--focal " << focal_pixels << std::endl
            << "--intrinsics " << sKmatrix << std::endl
            << "--camera_model " << i_User_camera_model << std::endl
            << "--group_camera_model " << b_Group_camera_model << std::endl
            << "--cameraCalibrationFile " << sCalibrationFile << std::endl
            << "--writeCameraFiles " << cmd.used('G') << std::endl
            << "--parentDirAsRoot " << cmd.used('B') << std::endl;

  // Expected properties for each image
  double width = -1, height = -1, focal = -1, ppx = -1, ppy = -1;

  const EINTRINSIC e_User_camera_model = EINTRINSIC(i_User_camera_model);

  if (!stlplus::folder_exists(sImageDir))
  {
    std::cerr << "\nThe input directory doesn't exist" << std::endl;
    return EXIT_FAILURE;
  }

  if (sOutputDir.empty())
  {
    std::cerr << "\nInvalid output directory" << std::endl;
    return EXIT_FAILURE;
  }

  if (!stlplus::folder_exists(sOutputDir))
  {
    if (!stlplus::folder_create(sOutputDir))
    {
      std::cerr << "\nCannot create output directory" << std::endl;
      return EXIT_FAILURE;
    }
  }

  // Try to get camera intrinsics from given calibration file
  std::map<std::string, std::shared_ptr<openMVG::cameras::IntrinsicBase> > intrinsic_list;
  if (!sCalibrationFile.empty())
  {
    intrinsic_list = parseCalibrationFile(sCalibrationFile);
  }
  else if (!sKmatrix.empty() && !checkIntrinsicStringValidity(sKmatrix, focal, ppx, ppy))
  {
    std::cerr << "\nInvalid K matrix input" << std::endl;
    return EXIT_FAILURE;
  }

  if (!sKmatrix.empty() && focal_pixels != -1.0)
  {
    std::cerr << "\nCannot combine -f and -k options" << std::endl;
    return EXIT_FAILURE;
  }

  std::vector<Datasheet> vec_database;
  if (!sfileDatabase.empty())
  {
    if (!parseDatabase(sfileDatabase, vec_database))
    {
      std::cerr << "\nInvalid input database: " << sfileDatabase << ", please specify a valid file." << std::endl;
      return EXIT_FAILURE;
    }
  }

  // make sure that only one pose prior is used
  if (cmd.used('P') && (cmd.used('T') || cmd.used('R')))
  {
    std::cerr << "Error: --use_traj_prior_rot/--use_traj_prior_center and --use_pose_prior cannot be used at the same time." << std::endl;
    return EXIT_FAILURE;
  }

  // Check if prior weights are given
  if (cmd.used('P') && !sPriorWeights.empty())
  {
    prior_w_info = checkPriorWeightsString(sPriorWeights);
  }
  else if (cmd.used('P'))
  {
    prior_w_info.first = true;
  }

  // read images
  std::vector<std::string> vec_image = stlplus::folder_files(sImageDir); // contains a list of filenames (1234.567.png, 678.989.png, ...)
  std::sort(vec_image.begin(), vec_image.end());

  // read poses
  std::vector<geometry::Pose3> vec_poses;
  std::vector<geometry::Pose3>::const_iterator iter_poses;

  //Setup the camera type and the appropriate camera reader
  bool (*fcnReadCamPtr)(const std::string &, PinholeCamera &);
  std::string suffix;
  std::map< std::string, std::pair<Mat3, Vec3>> map_Rt_gt;
  std::map<size_t, PinholeCamera, std::less<size_t>,
    Eigen::aligned_allocator<std::pair<const size_t, PinholeCamera>>> map_Cam_gt;

  if (!sGroundTruthPath.empty())
  {
    switch (i_gt_type)
    {
      case 1:
        // load NLK format
        vec_poses = loadPosesNLK(sGroundTruthPath);
        break;
      case 2:
      {
        // load Strecha's camera files
        std::cout << "Using Strechas Camera" << std::endl;
        fcnReadCamPtr = &read_Strecha_Camera;
        suffix = "camera";
        std::vector<std::string> vec_fileNames;
        readGt(fcnReadCamPtr, sGroundTruthPath, suffix, vec_fileNames, map_Rt_gt, map_Cam_gt);
        std::cout << map_Cam_gt.size() << " GT cameras have been found." << std::endl;
        // convert to vector, the first element of map map_Cam_gt needs to correspond to the index of vec_image
        // this is enforced by ordering both containers by filename of images and camera files
        if (map_Cam_gt.empty())
        {
          std::cout << "No GT camera files are found. Aborting..." << std::endl;
          return EXIT_FAILURE;
        }
        for (int i = 0; i < vec_image.size(); i++)
        {
          PinholeCamera cam = map_Cam_gt[i];
          Pose3 pose(cam._R, cam._C);
          vec_poses.push_back(pose);
        }
        if (!intrinsic_list.empty())
        {
          intrinsic_list.clear();
          std::cout << "Since GT files (--gtType = 2) are provided, --cameraCalibrationFile will be ignored." << std::endl;
        }
        // use intrinsics from camera files
        i_User_camera_model = PINHOLE_CAMERA_BROWN;
        break;
      }
      case 3:
        // load TUM format
        vec_poses = loadPosesTUM(sGroundTruthPath, vec_image);
        break;
      default:
        std::cerr << "Unsupported ground truth type." << std::endl;
        return EXIT_FAILURE;
    }
  }

  if (vec_image.size() != vec_poses.size() && !sGroundTruthPath.empty())
  {
    std::cerr << "Number of poses is not equal to number of images!" << std::endl;
    return EXIT_FAILURE;
  }

  // Configure an empty scene with Views and their corresponding cameras and poses
  SfM_Data sfm_data;
  Hash_Map<std::string, IndexT> map_filename_id;

  // append new data to sfm_data file (if it exists)
  if (cmd.used('A'))
  {
    std::string sfm_data_file_name = stlplus::create_filespec(sOutputDir, sSfmDataName);
    if (Load(sfm_data, sfm_data_file_name.c_str(), ESfM_Data(VIEWS | EXTRINSICS | INTRINSICS)))
    {
      std::cout << "Existing sfm_data file " << sfm_data_file_name << " loaded. New data will be appended."
                << std::endl;

      // generate mapping between filename and view id to avoid duplicating entries
      for (auto & view : sfm_data.GetViews())
        map_filename_id[stlplus::filename_part(view.second->s_Img_path)] = view.first;
    }
    else
      std::cout << "--appendData flag set but sfm_data file does not exist. New file will be created." << std::endl;
  }

  Views &views = sfm_data.views;
  Intrinsics &intrinsics = sfm_data.intrinsics;
  Poses &poses = sfm_data.poses;
  std::vector<std::string>::const_iterator iter_image;

  C_Progress_display my_progress_bar(vec_image.size(), std::cout, "\n- Image listing -\n");
  std::ostringstream error_report_stream;
  for (iter_image = vec_image.begin(), iter_poses = vec_poses.begin();
       iter_image != vec_image.end(); ++iter_image, ++my_progress_bar, ++iter_poses)
  {
    int index = std::distance(vec_image.cbegin(), iter_image);
    // Read meta data to fill camera parameter (w,h,focal,ppx,ppy) fields.
    width = height = ppx = ppy = focal = -1.0;

    const std::string sImageFilename = stlplus::create_filespec(sImageDir, *iter_image);
    const std::string sImFilenamePart = stlplus::filename_part(sImageFilename);

    // the camera name is the name of the image directory
    std::string camname;
    std::vector<std::string> values;
    values = stlplus::folder_elements(sImageDir);
    camname = values[values.size()-1];

    // Test if the image format is supported:
    if (openMVG::image::GetFormat(sImageFilename.c_str()) == openMVG::image::Unknown)
    {
      error_report_stream << sImFilenamePart << ": Unkown image file format." << "\n";
      continue; // image cannot be opened
    }

    if (sImFilenamePart.find("mask.png") != std::string::npos || sImFilenamePart.find("_mask.png") != std::string::npos)
    {
      error_report_stream << sImFilenamePart << " is a mask image" << "\n";
      continue;
    }

    ImageHeader imgHeader;
    if (!openMVG::image::ReadImageHeader(sImageFilename.c_str(), &imgHeader))
    {
      continue;
    } // image cannot be read

    width = imgHeader.width;
    height = imgHeader.height;

    if (!intrinsic_list.empty())
    {
      if (intrinsic_list.find(camname) != intrinsic_list.end())
      {
        std::vector<double> camera_params = intrinsic_list[camname]->getParams();
        focal = camera_params[0];
        ppx = camera_params[1];
        ppy = camera_params[2];
      }
      else
      {
        std::cerr << "Camera was not found in calibration file!" << std::endl;
        return EXIT_FAILURE;
      }
    }
    else if (!map_Cam_gt.empty() && map_Cam_gt.find(index) != map_Cam_gt.end())
    {
      PinholeCamera cam = map_Cam_gt[index];
      focal = (cam._K(0,0) + cam._K(1,1)) / 2.0;
      ppx = cam._K(0,2);
      ppy = cam._K(1,2);
    }
    else
    {
      ppx = width / 2.0;
      ppy = height / 2.0;
    }

    // Consider the case where the focal is provided manually
    if (sKmatrix.size() > 0) // Known user calibration K matrix
    {
      if (!checkIntrinsicStringValidity(sKmatrix, focal, ppx, ppy))
      {
        focal = -1.0;
      }
    }
    else if (focal_pixels != -1) // User provided focal length value
    {
      focal = focal_pixels;
    }

    // If not manually provided or wrongly provided
    if (focal == -1)
    {
      std::unique_ptr<Exif_IO> exifReader(new Exif_IO_EasyExif);
      exifReader->open(sImageFilename);

      const bool bHaveValidExifMetadata = exifReader->doesHaveExifInfo() && !exifReader->getModel().empty();

      if (bHaveValidExifMetadata) // If image contains meta data
      {
        const std::string sCamModel = exifReader->getModel();

        // Handle case where focal length is equal to 0
        if (exifReader->getFocal() == 0.0f)
        {
          error_report_stream << stlplus::basename_part(sImageFilename) << ": Focal length is missing." << "\n";
          focal = -1.0;
        }
        else
          // Create the image entry in the list file
        {
          Datasheet datasheet;
          if (getInfo(sCamModel, vec_database, datasheet))
          {
            // The camera model was found in the database so we can compute it's approximated focal length
            const double ccdw = datasheet.sensorSize_;
            focal = std::max(width, height) * exifReader->getFocal() / ccdw;
          }
          else
          {
            error_report_stream << stlplus::basename_part(sImageFilename) << "\" model \"" << sCamModel
                                << "\" doesn't exist in the database" << "\n"
                                << "Please consider add your camera model and sensor width in the database." << "\n";
          }
        }
      }
    }

    Mat3 K;
    K << focal, 0, ppx, 0, focal, ppy, 0, 0, 1;

    // Build intrinsic parameter related to the view
    std::shared_ptr<IntrinsicBase> intrinsic;

    if (!intrinsic_list.empty() && intrinsic_list.find(camname) != intrinsic_list.end() && !camname.empty())
    {
      intrinsic = intrinsic_list[camname];
    }
    else if (focal > 0 && ppx > 0 && ppy > 0 && width > 0 && height > 0)
    {
      // Create the desired camera type
      switch (e_User_camera_model)
      {
        case PINHOLE_CAMERA:
          intrinsic = std::make_shared<Pinhole_Intrinsic>(width, height, focal, ppx, ppy);
          break;
        case PINHOLE_CAMERA_RADIAL1:
          intrinsic = std::make_shared<Pinhole_Intrinsic_Radial_K1>(width, height, focal, ppx, ppy,
                                                                    0.0); // setup no distortion as initial guess
          break;
        case PINHOLE_CAMERA_RADIAL3:
          intrinsic = std::make_shared<Pinhole_Intrinsic_Radial_K3>(width, height, focal, ppx, ppy, 0.0, 0.0,
                                                                    0.0);  // setup no distortion as initial guess
          break;
        case PINHOLE_CAMERA_BROWN:
          intrinsic = std::make_shared<Pinhole_Intrinsic_Brown_T2>(width, height, focal, ppx, ppy, 0.0, 0.0, 0.0, 0.0,
                                                                   0.0); // setup no distortion as initial guess
          break;
        case PINHOLE_CAMERA_FISHEYE:
          intrinsic = std::make_shared<Pinhole_Intrinsic_Fisheye>(width, height, focal, ppx, ppy, 0.0, 0.0, 0.0,
                                                                  0.0); // setup no distortion as initial guess
          break;
        case CAMERA_SPHERICAL:
          intrinsic = std::make_shared<Intrinsic_Spherical>(width, height);
          break;
        default:
          std::cerr << "Error: unknown camera model: " << (int) e_User_camera_model << std::endl;
          return EXIT_FAILURE;
      }
    }

    // Check if view already exists (when using option '-A'): if yes, don't add the view
    if (map_filename_id.find(sImFilenamePart) == map_filename_id.end())
    {
      // Build the view corresponding to the image
      const std::pair<bool, Vec3> gps_info = checkGPS(sImageFilename, i_GPS_XYZ_method);
      if (cmd.used('T') || cmd.used('P'))
      {
        ViewPriors v(sImageFilename, views.size(), views.size(), views.size(), width, height);

        // Add intrinsic related to the image (if any)
        if (intrinsic == nullptr)
        {
          //Since the view have invalid intrinsic data
          // (export the view, with an invalid intrinsic field value)
          v.id_intrinsic = UndefinedIndexT;
        }
        else
        {
          // Add the defined intrinsic to the sfm_container
          intrinsics[v.id_intrinsic] = intrinsic;
        }

        // if GPS prior is used -> position only
        if (gps_info.first && cmd.used('P'))
        {
          v.b_use_pose_center_ = true;
          v.pose_center_ = gps_info.second;
          // prior weights
          if (prior_w_info.first == true)
          {
            v.center_weight_ = prior_w_info.second;
          }
        }
        else // if cmd.used('T') and/or cmd.used('R') is used -> position and/or orientation
        {
          if (cmd.used('T'))
          {
            v.b_use_pose_center_ = true;
            v.pose_center_ = iter_poses->center();
          }
          if (cmd.used('R'))
          {
            v.b_use_pose_rotation_ = true;
            v.pose_rotation_ = iter_poses->rotation();
          }
        }

        views[v.id_view] = std::make_shared<ViewPriors>(v);
      }
      else
      {
        View v(sImageFilename, views.size(), views.size(), views.size(), width, height);

        // Add intrinsic related to the image (if any)
        if (intrinsic == nullptr)
        {
          //Since the view have invalid intrinsic data
          // (export the view, with an invalid intrinsic field value)
          v.id_intrinsic = UndefinedIndexT;
        }
        else
        {
          // Add the defined intrinsic to the sfm_container
          intrinsics[v.id_intrinsic] = intrinsic;
        }

        views[v.id_view] = std::make_shared<View>(v);
      }

      if (!sGroundTruthPath.empty())
        poses[poses.size()] = *iter_poses;
    }

    // Write poses as camera files
    if (cmd.used('G') && !sGroundTruthPath.empty())
    {
      if (e_User_camera_model != CAMERA_SPHERICAL)
      {
        PinholeCamera camGT(K, iter_poses->rotation(), iter_poses->translation());
        std::string sCameraFileName = sImFilenamePart + ".bin";
        save(stlplus::create_filespec(sOutputDir, sCameraFileName).c_str(), camGT);
      }
    }
  }

  // organize paths and filenames in sfm_data file
  // set s_root_path when a new sfm_data file was created
  if (sfm_data.s_root_path.empty())
  {
    // if parent folder of image directory should be root of sfm_data
    if (cmd.used('B'))
    {
      // first, remove the folder separator (if there is one at the end of sImageDir)
      // second, take the folder part (which is everything till the last separator) as root path
      sfm_data.s_root_path = stlplus::folder_part(stlplus::folder_remove_end_separator(sImageDir));

      for (auto &view : sfm_data.GetViews())
      {
        std::string img_path = stlplus::folder_to_relative_path(sfm_data.s_root_path, stlplus::folder_part(view.second->s_Img_path)) + stlplus::filename_part(view.second->s_Img_path);
        view.second->s_Img_path = img_path;
      }
    }
    else
    {
      sfm_data.s_root_path = sImageDir;

      for (auto &view : sfm_data.GetViews())
        view.second->s_Img_path = stlplus::filename_part(view.second->s_Img_path);
    }
  }
  else // set s_root_path when a data was appended to sfm_data file
  {
    // find common root directory between current sfm_data and new images
    const std::string common_root_dir = FindCommonRootDir(sfm_data.s_root_path, sImageDir);

    for (auto &view : sfm_data.GetViews())
    {
      std::string full_path = "";
      if (map_filename_id.find(stlplus::filename_part(view.second->s_Img_path)) != map_filename_id.end())
      {
        full_path = stlplus::folder_append_separator(sfm_data.s_root_path) + view.second->s_Img_path;
      }
      else
        full_path = view.second->s_Img_path;

      full_path = stlplus::folder_part(full_path);

      view.second->s_Img_path = stlplus::folder_to_relative_path(common_root_dir, full_path) + stlplus::filename_part(view.second->s_Img_path);
    }
    sfm_data.s_root_path = common_root_dir;
  }

  // Display saved warning & error messages if any.
  if (!error_report_stream.str().empty())
  {
    std::cerr << "\nWarning & Error messages:" << std::endl << error_report_stream.str() << std::endl;
  }

  // Group camera that share common properties if desired (leads to more faster & stable BA).
  if (b_Group_camera_model)
  {
    GroupSharedIntrinsics(sfm_data);
  }

  // Store SfM_Data views & intrinsic data
  if (!Save(sfm_data, stlplus::create_filespec(sOutputDir, sSfmDataName).c_str(),
            ESfM_Data(VIEWS | INTRINSICS | EXTRINSICS)))
  {
    return EXIT_FAILURE;
  }

  std::cout << std::endl << "SfMInit_ImageListing report:\n" << "listed #File(s): " << vec_image.size() << "\n"
            << "usable #File(s) listed in sfm_data: " << sfm_data.GetViews().size() << "\n" << "listed #Pose(s): "
            << vec_poses.size() << "\n" << "usable #Intrinsic(s) listed in sfm_data: "
            << sfm_data.GetIntrinsics().size() << std::endl;

  return EXIT_SUCCESS;
}
