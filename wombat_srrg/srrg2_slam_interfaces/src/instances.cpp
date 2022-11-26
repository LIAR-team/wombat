#include "instances.h"
#include "wombat_srrg/srrg2_slam_interfaces/mapping/merger_correspondence_homo.h"
#include "wombat_srrg/srrg2_slam_interfaces/registration/aligners/aligner_slice_processor.h"
#include "wombat_srrg/srrg2_slam_interfaces/registration/aligners/aligner_termination_criteria.h"
#include "wombat_srrg/srrg2_slam_interfaces/registration/aligners/multi_aligner.h"
#include "wombat_srrg/srrg2_slam_interfaces/registration/local_map_selectors/local_map_selector_breadth_first.h"
#include "wombat_srrg/srrg2_slam_interfaces/registration/local_map_selectors/local_map_selector_user_defined.h"
#include "wombat_srrg/srrg2_slam_interfaces/registration/loop_detector/multi_loop_detector_brute_force.h"
#include "wombat_srrg/srrg2_slam_interfaces/registration/loop_detector/multi_loop_detector_hbst.h"
#include "wombat_srrg/srrg2_slam_interfaces/registration/relocalization/multi_relocalizer.h"
#include "wombat_srrg/srrg2_slam_interfaces/system/multi_graph_slam.h"
#include "wombat_srrg/srrg2_slam_interfaces/trackers/multi_tracker.h"
#include "wombat_srrg/srrg2_slam_interfaces/trackers/tracker_slice_processor.h"


// ds reduce digital footprint and enforce linker sanity
/*
#define BOSS_REGISTER_CLASS(CLASS_)                 \
   CLASS_ dummy_##CLASS_;                            \
   BOSS_REGISTER_CLASS(CLASS_)
*/
namespace srrg2_slam_interfaces {
  using namespace srrg2_core;

  void srrg2_slam_interfaces_registerTypes() {
    // srrg2_solver::solver_registerTypes();
    // srrg2_solver::variables_and_factors_2d_registerTypes();
    // srrg2_solver::variables_and_factors_3d_registerTypes();
    BOSS_REGISTER_CLASS(TrackerInputHandle);
    BOSS_REGISTER_CLASS(TrackerReportRecord);

    BOSS_REGISTER_CLASS(MultiAligner2D);
    BOSS_REGISTER_CLASS(RawDataPreprocessorOdom2D);
    BOSS_REGISTER_CLASS(AlignerSliceOdom2DPrior);
    BOSS_REGISTER_CLASS(AlignerSliceOdom3DPrior);
    BOSS_REGISTER_CLASS(AlignerSliceMotionModel2D);
    BOSS_REGISTER_CLASS(AlignerSliceMotionModel3D);
    BOSS_REGISTER_CLASS(AlignerTerminationCriteriaStandard2D);
    BOSS_REGISTER_CLASS(AlignerTerminationCriteriaStandard3DQR);
    BOSS_REGISTER_CLASS(MergerCorrespondencePointNormal2f);
    BOSS_REGISTER_CLASS(MergerCorrespondencePointIntensityDescriptor3f);

    BOSS_REGISTER_CLASS(TrackerSliceProcessorEstimationBuffer2D);
    BOSS_REGISTER_CLASS(TrackerSliceProcessorEstimationBuffer3D);
    BOSS_REGISTER_CLASS(TrackerSliceProcessorPriorOdom2D);
    BOSS_REGISTER_CLASS(TrackerSliceProcessorPriorOdom3D);
    BOSS_REGISTER_CLASS(RawDataPreprocessorOdom3D);
    BOSS_REGISTER_CLASS(RawDataPreprocessorTrackerEstimate2D);
    BOSS_REGISTER_CLASS(RawDataPreprocessorTrackerEstimate3D);
    BOSS_REGISTER_CLASS(MultiTracker2D);
    BOSS_REGISTER_CLASS(MultiTracker3D);
    BOSS_REGISTER_CLASS(MultiAligner3D);
    BOSS_REGISTER_CLASS(MultiAligner3DQR);

    BOSS_REGISTER_CLASS(MotionModelConstantVelocity2D);
    BOSS_REGISTER_CLASS(MotionModelConstantVelocity3D);

    BOSS_REGISTER_CLASS(LocalMap2D);
    BOSS_REGISTER_CLASS(LocalMap3D);
    BOSS_REGISTER_CLASS(LocalMapSplittingCriterionDistance2D);
    BOSS_REGISTER_CLASS(LocalMapSplittingCriterionDistance3D);
    BOSS_REGISTER_CLASS(LocalMapSplittingCriterionRotation3D);
    BOSS_REGISTER_CLASS(LocalMapSplittingCriterionViewpoint3D);
    BOSS_REGISTER_CLASS(LocalMapSplittingCriterionVisibility3D);

    BOSS_REGISTER_CLASS(LoopClosure2D);
    BOSS_REGISTER_CLASS(LoopClosure3D);
    BOSS_REGISTER_CLASS(MultiGraphSLAM2D);
    BOSS_REGISTER_CLASS(MultiGraphSLAM3D);
    BOSS_REGISTER_CLASS(MultiRelocalizer2D);
    BOSS_REGISTER_CLASS(MultiRelocalizer3D);
    BOSS_REGISTER_CLASS(MultiLoopDetectorBruteForce2D);
    BOSS_REGISTER_CLASS(MultiLoopDetectorBruteForce3D);
    BOSS_REGISTER_CLASS(LocalMapSelectorBreadthFirst2D);
    BOSS_REGISTER_CLASS(LocalMapSelectorBreadthFirst3D);
    BOSS_REGISTER_CLASS(MultiLoopDetectorHBST2D);
    BOSS_REGISTER_CLASS(MultiLoopDetectorHBST3D);
    BOSS_REGISTER_CLASS(MapListener);

    // TODO recover this when we can convert ros msg camera matrix to 2D matrix
    //    BOSS_REGISTER_CLASS(InitializerCamera2D);
    BOSS_REGISTER_CLASS(InitializerCamera3D);
    BOSS_REGISTER_CLASS(InitializerStereoCamera3D);
    BOSS_REGISTER_CLASS(MultiInitializer);

    // messages
    BOSS_REGISTER_CLASS(LocalMapMessage2D);
    BOSS_REGISTER_CLASS(NodeUpdateMessage2D);
    BOSS_REGISTER_CLASS(FactorMessageSE2);

    BOSS_REGISTER_CLASS(LocalMapMessage3D);
    BOSS_REGISTER_CLASS(NodeUpdateMessage3D);
    BOSS_REGISTER_CLASS(FactorMessageSE3);
  }

} // namespace srrg2_slam_interfaces
