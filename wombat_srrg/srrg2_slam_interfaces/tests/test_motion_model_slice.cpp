#include <wombat_srrg/srrg_test/test_helper.hpp>

#include "wombat_srrg/srrg_geometry/geometry2d.h"
#include "wombat_srrg/srrg_geometry/geometry3d.h"
#include "wombat_srrg/srrg2_slam_interfaces/instances.h"

using namespace srrg2_slam_interfaces;
using namespace srrg2_core;

// NOTE: below tests expect the SLAM processing sequence as of 2019/10/18
// if the sequence is modified (e.g. different order of set/populate/merge),
// these tests might fail and the motion model slice has to be adjusted!

// modified tracker for testing only
class MockedMultiTracker3D : public MultiTracker3D
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using BaseType = MultiTracker3D;

  // set new empty scene and change into tracking state
  void setScene(SceneContainerType * scene)
  {
    BaseType::setScene(scene);
    _status = TrackerBase::Tracking;
  }

  // adaption always leads to tracking state (with prior slice only)
  void preprocessRawData() override
  {
    BaseType::preprocessRawData();
    _status = TrackerBase::Tracking;
  }

  // align does not overwrite the tracker estimate based on motion model
  void align();
};
using MockedMultiTracker3DPtr = std::shared_ptr<MockedMultiTracker3D>;

// readability
void setup(MockedMultiTracker3DPtr & tracker_, MultiAligner3DQRPtr & aligner_);

int main(int argc_, char ** argv_)
{
  return srrg2_test::runTests(argc_, argv_);
}

TEST(MultiAlignerSliceMotionModel3D, Random)
{
  // allocate tracker and aligner modules
  MockedMultiTracker3DPtr tracker;
  MultiAligner3DQRPtr aligner;
  setup(tracker, aligner);
  // local map container, starting pose and dummy measurement
  PropertyContainerDynamic local_map;
  Isometry3f previous_robot_in_local_map(Isometry3f::Identity());
  BaseSensorMessagePtr message(new BaseSensorMessage());

  // process initial measurement
  tracker->setScene(&local_map);
  tracker->populateScene(local_map);
  tracker->setRobotInLocalMap(previous_robot_in_local_map);
  tracker->setRawData(message);
  tracker->compute();

  // simulate motion
  Isometry3f motion_previous(Isometry3f::Identity());
  for (size_t i = 0; i < 10; ++i) {
    srand(i);
    Isometry3f motion(Isometry3f::Identity());
    motion.translation() += Vector3f::Random();
    motion.rotate(AngleAxisf(rand() * M_PI, Vector3f::UnitX()));
    motion.rotate(AngleAxisf(rand() * M_PI, Vector3f::UnitY()));
    motion.rotate(AngleAxisf(rand() * M_PI, Vector3f::UnitZ()));

    // update tracker for new pose
    const Isometry3f robot_in_local_map(previous_robot_in_local_map * motion);
    tracker->setRobotInLocalMap(robot_in_local_map);
    tracker->setRawData(message);
    tracker->compute();

    // the motion model should correspond to the previous true motion
    ASSERT_EQ(tracker->status(), MultiTracker3D::Status::Tracking);

    // recall that the aligner operates in flipped transform (i.e. moving -> fixed)
    const Vector6f error = geometry3d::t2v(aligner->movingInFixed() * motion_previous);
    //      geometry3d::t2v(motion_previous.inverse()) - geometry3d::t2v(aligner->movingInFixed());

    // no noise, no error
    ASSERT_LT(error.norm(), 1e-5 /*float inverses*/);
    previous_robot_in_local_map = robot_in_local_map;
    motion_previous             = motion;
  }
}

TEST(MultiAlignerSliceMotionModel3D, LocalMapCreation)
{
  // allocate tracker and aligner modules
  MockedMultiTracker3DPtr tracker;
  MultiAligner3DQRPtr aligner;
  setup(tracker, aligner);

  // local map container, starting pose and dummy measurement
  PropertyContainerDynamic local_map;
  Isometry3f previous_estimate(Isometry3f::Identity());
  BaseSensorMessagePtr message(new BaseSensorMessage());

  // process initial measurement
  tracker->populateScene(local_map);
  tracker->setScene(&local_map);
  tracker->setRobotInLocalMap(previous_estimate);
  tracker->setRawData(message);
  tracker->compute();

  // simulate motion
  Isometry3f motion_previous(Isometry3f::Identity());
  for (size_t i = 0; i < 100; ++i) {
    srand(i);
    Isometry3f motion(Isometry3f::Identity());
    motion.translation() += Vector3f::Random();
    motion.rotate(AngleAxisf(rand() * M_PI, Vector3f::UnitX()));
    motion.rotate(AngleAxisf(rand() * M_PI, Vector3f::UnitY()));
    motion.rotate(AngleAxisf(rand() * M_PI, Vector3f::UnitZ()));
    Isometry3f current_robot_in_local_map(previous_estimate * motion);

    // advance tracker in current sequence (as in SLAM)
    tracker->setRawData(message);
    tracker->preprocessRawData();
    tracker->align();

    // simulate multiple consecutive new local map creations
    if (i % 10 == 0) {
      current_robot_in_local_map = motion;
      PropertyContainerDynamic new_local_map;
      tracker->populateScene(new_local_map);
      tracker->setScene(&new_local_map);
    }
    tracker->setRobotInLocalMap(current_robot_in_local_map);
    tracker->merge();

    // the motion model should correspond to the previous true motion
    ASSERT_EQ(tracker->status(), MultiTracker3D::Status::Tracking);

    // recall that the aligner operates in flipped transform (i.e. moving -> fixed)
    const Vector6f error = geometry3d::t2v(aligner->movingInFixed() * motion_previous);

    // no noise, no error
    ASSERT_LT(error.norm(), 1e-5 /*float inverses*/);
    previous_estimate = current_robot_in_local_map;
    motion_previous   = motion;
  }
}

TEST(MultiAlignerSliceMotionModel3D, Relocalization)
{
  // allocate tracker and aligner modules
  MockedMultiTracker3DPtr tracker;
  MultiAligner3DQRPtr aligner;
  setup(tracker, aligner);

  // local map container, starting pose and dummy measurement
  PropertyContainerDynamic local_map_a;
  Isometry3f estimate_origin_a(Isometry3f::Identity());
  BaseSensorMessagePtr message(new BaseSensorMessage());

  // process initial measurement
  tracker->populateScene(local_map_a);
  tracker->setScene(&local_map_a);
  tracker->setRobotInLocalMap(estimate_origin_a);
  tracker->setRawData(message);
  tracker->compute();

  // simulate motion
  Isometry3f previous_estimate(Isometry3f::Identity());
  Isometry3f motion_previous(Isometry3f::Identity());
  for (size_t i = 0; i < 100; ++i) {
    srand(i);
    Isometry3f motion(Isometry3f::Identity());
    motion.translation() += Vector3f::Random();
    motion.rotate(AngleAxisf(rand() * M_PI, Vector3f::UnitX()));
    motion.rotate(AngleAxisf(rand() * M_PI, Vector3f::UnitY()));
    motion.rotate(AngleAxisf(rand() * M_PI, Vector3f::UnitZ()));
    Isometry3f current_estimate(previous_estimate * motion);
    const Isometry3f motion_transform(current_estimate * previous_estimate.inverse());

    // advance tracker in current sequence (as in SLAM)
    tracker->setRawData(message);
    tracker->preprocessRawData();
    tracker->setRobotInLocalMap(current_estimate);
    tracker->align();

    // simulate multiple relocalization - move into another local map (b)
    if (i % 10 == 0) {
      // new local map to relocalize in
      PropertyContainerDynamic local_map_b;
      Isometry3f estimate_origin_b(Isometry3f::Identity());
      estimate_origin_b.translation() += Vector3f::Random();
      estimate_origin_b.rotate(AngleAxisf(rand() * M_PI, Vector3f::UnitX()));
      estimate_origin_b.rotate(AngleAxisf(rand() * M_PI, Vector3f::UnitY()));
      estimate_origin_b.rotate(AngleAxisf(rand() * M_PI, Vector3f::UnitZ()));
      const Isometry3f local_map_a_in_b = estimate_origin_b.inverse() * estimate_origin_a;

      // estimate moves into local map B's coordinate system
      const Isometry3f current_estimate_in_a(current_estimate);
      current_estimate = local_map_a_in_b * current_estimate_in_a;

      // compute the relocalization motion required for above transition from previous
      const Isometry3f relocalization_transform = local_map_a_in_b * motion_transform;

      // verify relocalization mockup
      const Isometry3f previous_moved_in_b_1 = relocalization_transform * previous_estimate;
      const Isometry3f previous_moved_in_b_2 = (local_map_a_in_b * previous_estimate) * motion;
      const Isometry3f local_map_a_in_b_rev  = current_estimate * current_estimate_in_a.inverse();
      ASSERT_NEAR_EIGEN(previous_moved_in_b_1, current_estimate, 1e-4);
      ASSERT_NEAR_EIGEN(previous_moved_in_b_2, current_estimate, 1e-4);
      ASSERT_NEAR_EIGEN(local_map_a_in_b_rev, local_map_a_in_b, 1e-4);
      tracker->setClosure(CorrespondenceVector(), relocalization_transform, current_estimate);
      tracker->setScene(&local_map_b);
    }
    tracker->setRobotInLocalMap(current_estimate);
    tracker->merge();

    // the motion model should correspond to the previous true motion
    ASSERT_EQ(tracker->status(), MultiTracker3D::Status::Tracking);

    // recall that the aligner operates in flipped transform (i.e. moving -> fixed)
    const Vector6f error = geometry3d::t2v(aligner->movingInFixed() * motion_previous);

    // no noise, no error
    ASSERT_LT(error.norm(), 1e-4 /*float inverses*/);
    previous_estimate = current_estimate;
    motion_previous   = motion;
  }
}

// ---------------------------------------------------------------------------------
// test helper methods:
// align does not overwrite the tracker estimate based on motion model
void MockedMultiTracker3D::align()
{
  // configure aligner
  AlignerTypePtr aligner = param_aligner.value();
  aligner->setFixed(&_measurement_container);
  aligner->setMoving(&_clipped_scene_container);

  // compute relative transform between all fixed and moving slices
  aligner->compute();

  // store correspondences for merging
  aligner->storeCorrespondences();

  // evaluate tracker status based on aligner performance
  if (aligner->status() == AlignerBase::Success) {
    _status = TrackerBase::Tracking;

    // here the tracker estimate would be updated based on the alignment result

    // update tracker estimate with current one (changed externally)
    setRobotInLocalMap(_robot_in_local_map);
  } else {
    _status = TrackerBase::Lost;
  }
}

// tracker and aligner setup
void setup(MockedMultiTracker3DPtr & tracker_, MultiAligner3DQRPtr & aligner_)
{
  srrg2_slam_interfaces_registerTypes();

  // tracker configuration
  tracker_ = MockedMultiTracker3DPtr(new MockedMultiTracker3D());
  TrackerSliceProcessorEstimationBuffer3DPtr tracker_slice(
    new TrackerSliceProcessorEstimationBuffer3D());
  tracker_slice->param_measurement_slice_name.setValue("tracker_pose");
  tracker_slice->param_adaptor.setValue(
    RawDataPreprocessorTrackerEstimate3DPtr(new RawDataPreprocessorTrackerEstimate3D()));
  tracker_->param_slice_processors.pushBack(tracker_slice);

  // aligner configuration
  aligner_ = MultiAligner3DQRPtr(new MultiAligner3DQR());
  tracker_->param_aligner.setValue(aligner_);
  AlignerSliceMotionModel3DPtr aligner_slice(new AlignerSliceMotionModel3D);
  aligner_slice->param_motion_model.setValue(
    MotionModelConstantVelocity3DPtr(new MotionModelConstantVelocity3D()));
  aligner_slice->param_fixed_slice_name.setValue("tracker_pose");
  aligner_->param_slice_processors.pushBack(aligner_slice);

  // we're not performing an actual registration
  aligner_->param_min_num_inliers.setValue(0);
}