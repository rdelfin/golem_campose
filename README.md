# golem_campose

## Keypoint ordering

The keypoints in returned by the openpose_collector node are in a list with the following ordering:

0. Nose
1.  Neck
2.  Right Shoulder
3.  Right Elbow
4.  Right wrist
5.  Left Shoulder
6.  Left elbow
7.  Left wrist
8.  Right knee
10. Right ankle
11. Left hip
12. Left knee
13. Left ankle
14. Right eye
15. Left eye
16. Right ear
17. Left ear

The person_finder node grabs keypoints #0 and #1 and does a weighted average to get the x coordinate of the person and publish that.