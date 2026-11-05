# SDD ledger — plan: docs/superpowers/plans/2026-11-04-localization3d.md

User chose3D; existing autonomy authorization retained. Isolated standalone Git folder, no original source edits. Initial scope is 3D scan-to-prior-map baseline + odometry prediction + low-speed MPCC, numerical sensor simulation and explicitly labelled three-pose Gazebo visualization.

Pre-flight interfaces: sensor(local XYZ, known extrinsic) -> estimator(SE3); estimator planar extraction -> original seven-state MPCC; GT remains plant/sensor/evaluator-only. Both world and estimator use fixed map geometry; noise, quantization and inverse-crime limits must be reported.

Task1 complete: exact3Dfirst-hit/occlusion/no-return/rotation tests RED->GREEN. OriginalPCD hash7b4c61ea... verified,484319 retained XYZ points,303676 occupied .15m voxels, no planning geometry in sensor model.

Task2 complete: nonplanar6DoF registration, no-return rejection, raw-odom separation, tilted-frame adapter tests RED->GREEN; stationaryIMU/3Droad-frame tests RED->GREEN. Full scoped suite9/9 passes. First real-map static scan1266returns,41.6ms scan generation;948796surface faces built. 3DICP24iterations accepted, planeRMSE.00737m. Initial+.2/-.15/+.08m andsmallrotation seed corrected to.000501m position error in this same-geometry ideal benchmark, not real-sensor accuracy.

Builder check clarified velocity extraction: use estimated scan-to-scan motion with .4s low-pass lateral estimate,3-vector gyro, and mapXY/yaw-frame adapter (tested). No plant vy/gyro-z shortcut. Accelerometer recorded but this baseline does not double-integrate accelerometer or claim FAST-LIO.

Task3 in progress: 12 tests pass. First noisy-sensor probe exposed rawwheel measurement jumps at a hard velocity ceiling (QP infeasibility after.5s). Added tested.08s wheel lowpass; separated progress target1.5m/s from forward-state safety envelope1.8m/s. Original map/hard widths unchanged. New300-step6s probe passed every sensor/control step, all60scans accepted, GTcorridorviolation0; poseRMS.00198m in ideal same-geometry baseline. Full low-speed PC lap now running.

Task3 PC complete: full5603steps/112.06s lap,1121accepted3Dscans,GTcorridor0. Verified feedbackXY equals same-time estimate and differs fromGT. Independent polygon/grid audit passes. PoseRMS.001798m in ideal same-geometry benchmark; rawodomfinal2.8696m. No actualsensoraccuracyclaim.

Task4 in progress: Gazebo full-SE3replay plugin built, new three-circle display running. Topics contain marker_gt,marker_odo,marker_scan and vehicle_gt; overview camera ready. Different display radii preserve centroids and are not uncertainty radii. Jetson6s/300steps probe passed with same results and no corridor/solver failure; fresh fullJetsonlap running. Final code review pending.

Task3 Jetson complete: freshfullARM64lap5603steps112.06s,1121/1121scanupdatesaccepted,GTcorridor0. RetrievedarchiveSHA2569db16ad0...verified. Independentlyrepeatedpolygon/grid/estimatedfeedbackaudit PASS. PC/ARMpose differences<3.1e-10m. Jetsoncontrolmean25.19ms/max104.77ms, fullsequentialcyclemean74.00ms: explicitlyofflinenot20msrealtime.

Final review: noCritical/numericalleakagefinding. Importantvisible-ringcentreoffsetatlocalz1.6 reproducedbytest_display on tiltedposes (RED), fixedringgeometryatlocalorigin andheadingoffset, testGREEN. Also tieddisplayauditPASS to exactrunidentity. Fullsuite14/14pass. NewdisplayusesverifiedJetsonrecord;24Gazebo pose+visible-mesh-centrechecksPASS(<1.5e-8m). OlddisplayandPCauditarchived, noGT/numericalrerunneededforvisualfix. No deferred review findings.

Task4 complete: three-circleGazebo viewrunning, comparisonPNGinspected, READMEandreproductioncommandscomplete, originalsource/map/vehiclehashesunchanged. Usernoticedorangeodometrycrossinggraymap: explainedunconstrainedrawestimate/drift, confirmedGToriginalhardcontainmentindependently. Didnotcliprawodomtohideerror.
