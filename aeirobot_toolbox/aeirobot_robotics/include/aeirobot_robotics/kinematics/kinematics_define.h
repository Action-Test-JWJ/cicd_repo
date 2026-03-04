/**
 * @file kinematics_dynamics_define.h
 * @brief Alice4 로봇 기구학-동역학 ID 및 상수 정의
 *
 * @details
 *   - 모든 관절 및 링크 개수 정의 (ALL_JOINT_ID)
 *   - 좌/우 다리 시작 ID 정의 (ID_L_LEG_START, ID_R_LEG_START)
 */
#ifndef ALICE_KINEMATICS_DYNAMICS_KINEMATICS_DYNAMICS_DEFINE_H_
#define ALICE_KINEMATICS_DYNAMICS_KINEMATICS_DYNAMICS_DEFINE_H_

/** @brief 전체 조인트(링크) 개수 */
#define ALL_JOINT_ID (37)

/** @brief 오른쪽 다리 조인트 시작 인덱스 */
#define ID_R_LEG_START (14)

/** @brief 왼쪽 다리 조인트 시작 인덱스 */
#define ID_L_LEG_START (13)

#endif // ALICE_KINEMATICS_DYNAMICS_KINEMATICS_DYNAMICS_DEFINE_H_