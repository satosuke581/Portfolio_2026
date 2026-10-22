/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "ModularCANLib.h"
#include "usart.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
	ModularCANLib_DeviceInfo_Type *dev_info;
	ModularCANLib_DeviceParam_CANBoards_MCMD_Type *mcmd4_param;
}dcmotor;

typedef struct {
	float Kp, Ki, Kd, r, clip, integral, pos_pre;
}PID;

typedef struct {//コンドウのModularcanlib用の変数をまとめた構造体
	ModularCANLib_DeviceInfo_Type *dev_info_kondo;
	ModularCANLib_DeviceParam_CANBoards_Dynamixel_Kondo_Type *kondo_param;

}kondo;

typedef struct {
	uint8_t A;
	uint8_t B;
	uint8_t X;
	uint8_t Y;
	uint8_t LB;
	uint8_t RB;
	uint8_t BACK;
	uint8_t START;
	uint8_t Logicool;
	uint8_t LSP;
	uint8_t RSP;
	uint8_t left_hor;
	uint8_t left_ver;
	uint8_t right_hor;
	uint8_t LT;
	uint8_t RT;
	uint8_t right_ver;
	uint8_t hat_hor;
	uint8_t hat_ver;
	uint8_t sec_A;
	uint8_t sec_B;
	uint8_t sec_X;
	uint8_t sec_Y;
	uint8_t sec_LB;
	uint8_t sec_BACK;
	uint8_t sec_LT;
}command;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define size_cmd 7//入力コマンドのバイト数
#define Red 0
#define Blue 1

enum {
	mainarm_ = 0,
	subarm_ = 1,
	subdoor_ = 2,
	motor_x_ = 3,
	motor_y_ = 4,
	motor_z_ = 5,
	motor_th_ = 6,
	minimotor_ = 7,
	waterpomp_ = 8,
};

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
static volatile uint8_t g_rx_byte[size_cmd]; /* 割り込みが書き込むバッファ */
static volatile uint8_t g_cmd[size_cmd]; /* 最新のコマンド */

int RorB = -1;//0:赤, 1:青
bool isStructInited_main = false, isStructInited_sub = false, isSystemInited = false;
/* USER CODE END Variables */
osThreadId MainTaskHandle;
uint32_t mainTaskBuffer[ 512 ];
osStaticThreadDef_t mainTaskControlBlock;
osThreadId SubTaskHandle;
uint32_t SubTaskBuffer[ 512 ];
osStaticThreadDef_t SubTaskControlBlock;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
/**
 * @brief  UART受信完了コールバック
 *         受け取ったバイトを最新コマンドとして保存し、次のバイト受信を再開する。
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart) {
    if (huart->Instance == USART3) {
        memcpy((uint8_t *)g_cmd,(uint8_t *)g_rx_byte,size_cmd);
        /* 次の1バイトを受信予約（これを呼ばないと受信が止まる） */
        HAL_UART_Receive_IT(&huart3, (uint8_t *)g_rx_byte, size_cmd);
    }
}
//motor用の関数
void init_x(dcmotor *dcmotor_, int mcmd_num, int node_id_, int device_num_){//例：init_x(&motor_x, mcmd_1, 5, 0);
	// 接続先のMCMDの設定
	dcmotor_->dev_info = ModularCANLib_DeviceInfo_Struct_Init(ModularCANLib_DeviceType_CANBoards_MCMD4, mcmd_num);
	dcmotor_->dev_info->hcan = &hcan1; // 使用するCANバスを指定。ここではCAN1を使用
	dcmotor_->dev_info->node_id = node_id_; // MCMDのID, MCMD4.3の場合は基板上のスイッチでIDを指定できる（0〜7）

	// 制御パラメータの設定
	dcmotor_->mcmd4_param = ModularCANLib_CANBoards_MCMD_GetDeviceParam(dcmotor_->dev_info); // パラメーター構造体のポインタを取得
	dcmotor_->mcmd4_param->device_num = device_num_; // モーターの番号、0と1がある
	dcmotor_->mcmd4_param->ctrl_param.ctrl_type = MCMD_CTRL_DUTY;//MCMD_CTRL_POS;  // 位置制御を行う
	dcmotor_->mcmd4_param->ctrl_param.PID_param.kp = 0.07f;  // Pゲイン 0.10
	dcmotor_->mcmd4_param->ctrl_param.PID_param.ki = 0.0f;  // Iゲイン 0.0
	dcmotor_->mcmd4_param->ctrl_param.PID_param.kd = 0.0f;  // Dゲイン 0.0 (Dゲインは使いにくい)
	dcmotor_->mcmd4_param->ctrl_param.PID_param.kff = 0.0f;  // ffゲイン 0.0
	dcmotor_->mcmd4_param->ctrl_param.accel_limit = ACCEL_LIMIT_ENABLE;  // 出力duty比をclipするか
	dcmotor_->mcmd4_param->ctrl_param.accel_limit_size = 0.1f;  // 出力duty比をclipする場合の絶対値のmax値
	dcmotor_->mcmd4_param->ctrl_param.feedback = MCMD_FB_ENABLE;  // MCMDからF7にフィードバックを送信するか否か
	dcmotor_->mcmd4_param->ctrl_param.timup_monitor = TIMEUP_MONITOR_DISABLE;  // timeupは未実装なのでDISABLE。
	dcmotor_->mcmd4_param->enc_dir = MCMD_DIR_BC;  // Encoderの回転方向設定
	dcmotor_->mcmd4_param->rot_dir = MCMD_DIR_FW;  // モーターの回転方向設定
	dcmotor_->mcmd4_param->quant_per_unit = 1.0f/100.0f/4.0f;  // エンコーダーの分解能に対する制御値の変化量の割合

	// 原点サーチの設定
	dcmotor_->mcmd4_param->limit_sw_type = LIMIT_SW_NO;  // 原点サーチにNomaly Openedのスイッチを用いる
	dcmotor_->mcmd4_param->calib = CALIBRATION_DISABLE;  // 原点サーチを行う。
	dcmotor_->mcmd4_param->calib_duty = -0.03f;;  // 原点サーチ時のduty(右に初手設置)
	dcmotor_->mcmd4_param->offset = 0.0f;  // 原点のオフセット
	dcmotor_->mcmd4_param->fb_type = MCMD_FB_POS;  // 読み取った位置情報をF7にフィードバックする。
}
void init_y(dcmotor *dcmotor_, int mcmd_num, int node_id_, int device_num_){
	// 接続先のMCMDの設定
		dcmotor_->dev_info = ModularCANLib_DeviceInfo_Struct_Init(ModularCANLib_DeviceType_CANBoards_MCMD4, mcmd_num);
		dcmotor_->dev_info->hcan = &hcan1; // 使用するCANバスを指定。ここではCAN1を使用
		dcmotor_->dev_info->node_id = node_id_; // MCMDのID, MCMD4.3の場合は基板上のスイッチでIDを指定できる（0〜7）

		// 制御パラメータの設定
		dcmotor_->mcmd4_param = ModularCANLib_CANBoards_MCMD_GetDeviceParam(dcmotor_->dev_info); // パラメーター構造体のポインタを取得
		dcmotor_->mcmd4_param->device_num = device_num_; // モーターの番号、0と1がある
		dcmotor_->mcmd4_param->ctrl_param.ctrl_type = MCMD_CTRL_DUTY;//MCMD_CTRL_POS;  // 位置制御を行う
		dcmotor_->mcmd4_param->ctrl_param.PID_param.kp = 0.07f;  // Pゲイン 0.10
		dcmotor_->mcmd4_param->ctrl_param.PID_param.ki = 0.0f;  // Iゲイン 0.0
		dcmotor_->mcmd4_param->ctrl_param.PID_param.kd = 0.0f;  // Dゲイン 0.0 (Dゲインは使いにくい)
		dcmotor_->mcmd4_param->ctrl_param.PID_param.kff = 0.0f;  // ffゲイン 0.0
		dcmotor_->mcmd4_param->ctrl_param.accel_limit = ACCEL_LIMIT_ENABLE;  // 出力duty比をclipするか
		dcmotor_->mcmd4_param->ctrl_param.accel_limit_size = 0.1f;  // 出力duty比をclipする場合の絶対値のmax値
		dcmotor_->mcmd4_param->ctrl_param.feedback = MCMD_FB_ENABLE;  // MCMDからF7にフィードバックを送信するか否か
		dcmotor_->mcmd4_param->ctrl_param.timup_monitor = TIMEUP_MONITOR_DISABLE;  // timeupは未実装なのでDISABLE。

		if(RorB == Blue) dcmotor_->mcmd4_param->enc_dir = MCMD_DIR_FW;  // Encoderの回転方向設定(RedかBlueかで変える)
		else dcmotor_->mcmd4_param->enc_dir = MCMD_DIR_BC;
		if(RorB == Blue)dcmotor_->mcmd4_param->rot_dir = MCMD_DIR_FW;  // モーターの回転方向設定(RedかBlueかで変える)
		else dcmotor_->mcmd4_param->rot_dir = MCMD_DIR_BC;
		dcmotor_->mcmd4_param->quant_per_unit = 1.0f/100.0f/4.0f;  // エンコーダーの分解能に対する制御値の変化量の割合

		// 原点サーチの設定
		dcmotor_->mcmd4_param->limit_sw_type = LIMIT_SW_NO;  // 原点サーチにNomaly Openedのスイッチを用いる
		dcmotor_->mcmd4_param->calib = CALIBRATION_DISABLE;  // 原点サーチを行う。(手前にセッティング)
		dcmotor_->mcmd4_param->calib_duty = -0.03f;  // 原点サーチ時のduty
		dcmotor_->mcmd4_param->offset = 0.0f;  // 原点のオフセット
		dcmotor_->mcmd4_param->fb_type = MCMD_FB_POS;  // 読み取った位置情報をF7にフィードバックする。
}
void init_z(dcmotor *dcmotor_, int mcmd_num, int node_id_, int device_num_){
	// 接続先のMCMDの設定
		dcmotor_->dev_info = ModularCANLib_DeviceInfo_Struct_Init(ModularCANLib_DeviceType_CANBoards_MCMD4, mcmd_num);
		dcmotor_->dev_info->hcan = &hcan1; // 使用するCANバスを指定。ここではCAN1を使用
		dcmotor_->dev_info->node_id = node_id_; // MCMDのID, MCMD4.3の場合は基板上のスイッチでIDを指定できる（0〜7）

		// 制御パラメータの設定
		dcmotor_->mcmd4_param = ModularCANLib_CANBoards_MCMD_GetDeviceParam(dcmotor_->dev_info); // パラメーター構造体のポインタを取得
		dcmotor_->mcmd4_param->device_num = device_num_; // モーターの番号、0と1がある
		dcmotor_->mcmd4_param->ctrl_param.ctrl_type = MCMD_CTRL_DUTY;//MCMD_CTRL_POS;  // 位置制御を行う
		dcmotor_->mcmd4_param->ctrl_param.PID_param.kp = 0.07f;  // Pゲイン 0.10
		dcmotor_->mcmd4_param->ctrl_param.PID_param.ki = 0.0f;  // Iゲイン 0.0
		dcmotor_->mcmd4_param->ctrl_param.PID_param.kd = 0.0f;  // Dゲイン 0.0 (Dゲインは使いにくい)
		dcmotor_->mcmd4_param->ctrl_param.PID_param.kff = 0.0f;  // ffゲイン 0.0
		dcmotor_->mcmd4_param->ctrl_param.accel_limit = ACCEL_LIMIT_ENABLE;  // 出力duty比をclipするか
		dcmotor_->mcmd4_param->ctrl_param.accel_limit_size = 0.1f;  // 出力duty比をclipする場合の絶対値のmax値
		dcmotor_->mcmd4_param->ctrl_param.feedback = MCMD_FB_ENABLE;  // MCMDからF7にフィードバックを送信するか否か
		dcmotor_->mcmd4_param->ctrl_param.timup_monitor = TIMEUP_MONITOR_DISABLE;  // timeupは未実装なのでDISABLE。
		dcmotor_->mcmd4_param->enc_dir = MCMD_DIR_FW;  // Encoderの回転方向設定
		dcmotor_->mcmd4_param->rot_dir = MCMD_DIR_FW;  // モーターの回転方向設定
		dcmotor_->mcmd4_param->quant_per_unit = 1.0f/100.0f/4.0f;  // エンコーダーの分解能に対する制御値の変化量の割合

		// 原点サーチの設定
		dcmotor_->mcmd4_param->limit_sw_type = LIMIT_SW_NC;  // 原点サーチにNomaly Closedのスイッチを用いる
		dcmotor_->mcmd4_param->calib = CALIBRATION_DISABLE;  // 原点サーチを行う。
		dcmotor_->mcmd4_param->calib_duty = -0.027f;//-0.03f;  // 原点サーチ時のduty
		dcmotor_->mcmd4_param->offset = 0.0f;  // 原点のオフセット
		dcmotor_->mcmd4_param->fb_type = MCMD_FB_POS;  // 読み取った位置情報をF7にフィードバックする。
}
void init_th(dcmotor *dcmotor_, int mcmd_num, int node_id_, int device_num_){
	// 接続先のMCMDの設定
	dcmotor_->dev_info = ModularCANLib_DeviceInfo_Struct_Init(ModularCANLib_DeviceType_CANBoards_MCMD4, mcmd_num);
		dcmotor_->dev_info->hcan = &hcan1; // 使用するCANバスを指定。ここではCAN1を使用
		dcmotor_->dev_info->node_id = node_id_; // MCMDのID, MCMD4.3の場合は基板上のスイッチでIDを指定できる（0〜7）

		// 制御パラメータの設定
		dcmotor_->mcmd4_param = ModularCANLib_CANBoards_MCMD_GetDeviceParam(dcmotor_->dev_info); // パラメーター構造体のポインタを取得
		dcmotor_->mcmd4_param->device_num = device_num_; // モーターの番号、0と1がある
		dcmotor_->mcmd4_param->ctrl_param.ctrl_type = MCMD_CTRL_DUTY;//MCMD_CTRL_POS;  // 位置制御を行う
		dcmotor_->mcmd4_param->ctrl_param.PID_param.kp = 0.07f;  // Pゲイン 0.10
		dcmotor_->mcmd4_param->ctrl_param.PID_param.ki = 0.0f;  // Iゲイン 0.0
		dcmotor_->mcmd4_param->ctrl_param.PID_param.kd = 0.0f;  // Dゲイン 0.0 (Dゲインは使いにくい)
		dcmotor_->mcmd4_param->ctrl_param.PID_param.kff = 0.0f;  // ffゲイン 0.0
		dcmotor_->mcmd4_param->ctrl_param.accel_limit = ACCEL_LIMIT_ENABLE;  // 出力duty比をclipするか
		dcmotor_->mcmd4_param->ctrl_param.accel_limit_size = 0.1f;  // 出力duty比をclipする場合の絶対値のmax値
		dcmotor_->mcmd4_param->ctrl_param.feedback = MCMD_FB_ENABLE;  // MCMDからF7にフィードバックを送信するか否か
		dcmotor_->mcmd4_param->ctrl_param.timup_monitor = TIMEUP_MONITOR_DISABLE;  // timeupは未実装なのでDISABLE。
		dcmotor_->mcmd4_param->enc_dir = MCMD_DIR_FW;  // Encoderの回転方向設定
		dcmotor_->mcmd4_param->rot_dir = MCMD_DIR_FW;  // モーターの回転方向設定
		dcmotor_->mcmd4_param->quant_per_unit = 360.0f * 28.0f / 144.0f /2048.0f;  // エンコーダーの分解能に対する制御値の変化量の割合

		// 原点サーチの設定
		dcmotor_->mcmd4_param->limit_sw_type = LIMIT_SW_NC;  // 原点サーチにNomaly Closedのスイッチを用いる
		dcmotor_->mcmd4_param->calib = CALIBRATION_DISABLE;  // 原点サーチを行う。
		dcmotor_->mcmd4_param->calib_duty = -0.027f; // 原点サーチ時のduty
		dcmotor_->mcmd4_param->offset = 0.0f;  // 原点のオフセット
		dcmotor_->mcmd4_param->fb_type = MCMD_FB_POS;  // 読み取った位置情報をF7にフィードバックする。
}
void init_minimotor(dcmotor *dcmotor_, int mcmd_num, int node_id_, int device_num_){
	// 接続先のMCMDの設定
	dcmotor_->dev_info = ModularCANLib_DeviceInfo_Struct_Init(ModularCANLib_DeviceType_CANBoards_MCMD4, mcmd_num);
	dcmotor_->dev_info->hcan = &hcan1; // 使用するCANバスを指定。ここではCAN1を使用
	dcmotor_->dev_info->node_id = node_id_; // MCMDのID, MCMD4.3の場合は基板上のスイッチでIDを指定できる（0〜7）

	// 制御パラメータの設定
	dcmotor_->mcmd4_param = ModularCANLib_CANBoards_MCMD_GetDeviceParam(dcmotor_->dev_info); // パラメーター構造体のポインタを取得
	dcmotor_->mcmd4_param->device_num = device_num_; // モーターの番号、0と1がある
	dcmotor_->mcmd4_param->ctrl_param.ctrl_type = MCMD_CTRL_DUTY;  // DUTY制御を行う
	dcmotor_->mcmd4_param->ctrl_param.accel_limit = ACCEL_LIMIT_ENABLE;  // 出力duty比をclipするか
	dcmotor_->mcmd4_param->ctrl_param.accel_limit_size = 1.0f;  // 出力duty比をclipする場合の絶対値のmax値
	dcmotor_->mcmd4_param->ctrl_param.feedback = MCMD_FB_DISABLE;  // MCMDからF7にフィードバックを送信しない
	dcmotor_->mcmd4_param->ctrl_param.timup_monitor = TIMEUP_MONITOR_DISABLE;  // timeupは未実装なのでDISABLE。
	dcmotor_->mcmd4_param->rot_dir = MCMD_DIR_FW;  // モーターの回転方向設定

	// 原点サーチの設定
	dcmotor_->mcmd4_param->limit_sw_type = LIMIT_SW_NO;  // 原点サーチにNomaly Openedのスイッチを用いる
	dcmotor_->mcmd4_param->calib = CALIBRATION_DISABLE;  // 原点サーチを行わない
	dcmotor_->mcmd4_param->calib_duty = -0.03f;;  // 原点サーチ時のduty(右に初手設置)
	dcmotor_->mcmd4_param->offset = 0.0f;  // 原点のオフセット
	dcmotor_->mcmd4_param->fb_type = MCMD_FB_DUTY;  // 読み取った位置情報をF7にフィードバックする。
}

void DUTYmove(dcmotor *dcmotor_, float r){//status_enableの状態から
	MCMD_Calib(dcmotor_->dev_info);//status_calibに遷移
	MCMD_SetTarget(dcmotor_->dev_info, r);//目標値を設定
	MCMD_Control_Enable(dcmotor_->dev_info);//status_enableに遷移
}

float PIDmove(dcmotor *dcmotor_, PID *pid){//例move(&motor_x, 0.08f, 0.006f, 0.0f, 目標値);status_enableの状態から
	ModularCANLib_DeviceInfo_Type *dev_info2 = ModularCANLib_GetDeviceInfo_By_Name(dcmotor_->dev_info->name); // デバイス名でdevinfoを再取得
	ModularCANLib_MCMD_Feedback_Type feedback = Get_MCMD_Feedback(dev_info2);
	float pos = feedback.value;
	pid->integral += 0.001f * ((pid->r - pid->pos_pre) + (pid->r - pos)) / 2.0f;
	float der = (pid->pos_pre - pos) / 1000.0f;
	float d = pid->Kp * (pid->r - pos) + pid->Ki * pid->integral + pid->Kd * der;
	if(d>pid->clip) d = pid->clip;
	if(d<-pid->clip) d = -pid->clip;
	printf("%f\r\n", pos);
	DUTYmove(dcmotor_, d);
	pid->pos_pre = pos;
	return d;
}

float motor_pos(dcmotor *dcmotor_){
	ModularCANLib_DeviceInfo_Type *dev_info2 = ModularCANLib_GetDeviceInfo_By_Name(dcmotor_->dev_info->name); // デバイス名でdevinfoを再取得
	ModularCANLib_MCMD_Feedback_Type feedback = Get_MCMD_Feedback(dev_info2);
	return feedback.value;
}
//kondo用の関数
void init_kondo(kondo *kondo_, int mcmd_num, int id_){//引数は左からkondoのポインタ, enumで宣言したコンドウの番号, コンドウのid　例：init_kondo(&mainarm, kondo_1, 6);
	kondo_->dev_info_kondo = ModularCANLib_DeviceInfo_Struct_Init(ModularCANLib_DeviceType_CANBoards_DYNAMIXEL_KONDO, mcmd_num);
	osDelay(1);
	kondo_->dev_info_kondo->hcan = &hcan1; // CAN1を使用する場合
	kondo_->dev_info_kondo->node_id = 0;  // 基板のツマミが指す番号（基板にテプラで貼られているやつではない）の mod 8 をとったもの（0 ~ 7）
	kondo_->kondo_param = ModularCANLib_CANBoards_Dynamixel_Kondo_GetDeviceParam(kondo_->dev_info_kondo);

	kondo_->kondo_param->device_num = mcmd_num;
	kondo_->kondo_param->id = id_;                                              // KONDO のID（https://kikaiken.esa.io/posts/184#サーボ表面のテプラについて）
	kondo_->kondo_param->ctrl_param_kondo.ctrl_type = KONDO_CTRL_POS; // 制御モード
	kondo_->kondo_param->quant_per_degree = 1.0f;                             // 回転角 1 度 に対して使いたい値が何倍か
	kondo_->kondo_param->ctrl_param_kondo.feedback = KONDO_FB_ENABLE; // Feedback を有効化
	kondo_->kondo_param->fb_type = KONDO_FB_POS;                          // Feedback の種類
	kondo_->kondo_param->ctrl_param_kondo.trajectory_type = KONDO_TRAJECTORY_FORTHPOLY; // プロファイル軌道の種類
}

void start_kondo(kondo kondo_){//canやmcmdを初期化した後、コンドウをスタートさせるために使う
	Kondo_init(kondo_.dev_info_kondo);
	Kondo_Control_Enable(kondo_.dev_info_kondo);
}

void spin_mainarm(kondo kondo_, int *c){//メインアームをチップスターをとる方向に回転する。二個目の引数はこれまでに何回回転したかの参照わたし
	if(*c<5)Kondo_SetTarget_With_Time(kondo_.dev_info_kondo, ++*c*45, 0.7f);
	else if (*c == 5)Kondo_SetTarget_With_Time(kondo_.dev_info_kondo, ++*c*45.0f - 22.5f, 0.7f);//6回目の回転だけ30度回す。それ以降は回さない
}
void release_mainarm(kondo kondo_, int *c){
	if(*c <= 0) return;
	Kondo_SetTarget_With_Time(kondo_.dev_info_kondo, --*c*45, 0.7f);
}
//デコード用の関数
command decode_cmd(){
	uint8_t cmd_[size_cmd];
	memcpy(cmd_, (uint8_t *)g_cmd, size_cmd);
	unsigned long long cmd = 0;
	for(int i=0;i< size_cmd;i++){
		cmd += ((unsigned long long)cmd_[i])<<((6-i)*8);
	}
	command ret;
	ret.A  = (cmd >> 55) & 0x01;
	ret.B  = (cmd >> 54) & 0x01;
	ret.X  = (cmd >> 53) & 0x01;
	ret.Y  = (cmd >> 52) & 0x01;
	ret.LB  = (cmd >> 51) & 0x01;
	ret.RB  = (cmd >> 50) & 0x01;
	ret.BACK  = (cmd >> 49) & 0x01;
	ret.START  = (cmd >> 48) & 0x01;
	ret.Logicool  = (cmd >> 47) & 0x01;
	ret.LSP  = (cmd >> 46) & 0x01;
	ret.RSP = (cmd >> 45) & 0x01;
	ret.left_hor  = (cmd >> 37) & 0xFF;
	ret.left_ver  = (cmd >> 29) & 0xFF;
	ret.right_hor = (cmd >> 21) & 0xFF;
	ret.LT    = (cmd >> 20) & 0x01;
	ret.RT   = (cmd >> 19) & 0x01;
	ret.right_ver = (cmd >> 11) & 0xFF;
	ret.hat_hor     = (cmd >> 9) & 0x03;
	ret.hat_ver     = (cmd >> 7) & 0x03;
	ret.sec_A = (cmd >> 6) & 0x01;
	ret.sec_B = (cmd >> 5) & 0x01;
	ret.sec_X = (cmd >> 4) & 0x01;
	ret.sec_Y = (cmd >> 3) & 0x01;
	ret.sec_LB = (cmd >> 2) & 0x01;
	ret.sec_BACK = (cmd >> 1) & 0x01;
	ret.sec_LT = (cmd >> 0) & 0x01;
	return ret;
}
/* USER CODE END FunctionPrototypes */

void StartMainTask(void const * argument);
void StartSubTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* GetTimerTaskMemory prototype (linked to static allocation support) */
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/* USER CODE BEGIN GET_TIMER_TASK_MEMORY */
static StaticTask_t xTimerTaskTCBBuffer;
static StackType_t xTimerStack[configTIMER_TASK_STACK_DEPTH];

void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize )
{
  *ppxTimerTaskTCBBuffer = &xTimerTaskTCBBuffer;
  *ppxTimerTaskStackBuffer = &xTimerStack[0];
  *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
  /* place for user code */
}
/* USER CODE END GET_TIMER_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of MainTask */
  osThreadStaticDef(MainTask, StartMainTask, osPriorityNormal, 0, 512, mainTaskBuffer, &mainTaskControlBlock);
  MainTaskHandle = osThreadCreate(osThread(MainTask), NULL);

  /* definition and creation of SubTask */
  osThreadStaticDef(SubTask, StartSubTask, osPriorityIdle, 0, 512, SubTaskBuffer, &SubTaskControlBlock);
  SubTaskHandle = osThreadCreate(osThread(SubTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartMainTask */
/**
  * @brief  Function implementing the MainTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartMainTask */
void StartMainTask(void const * argument)
{
  /* USER CODE BEGIN StartMainTask */
	HAL_UART_Receive_IT(&huart3, (uint8_t *)g_rx_byte, size_cmd);//コントローラー入力の呼び出し開始
			command cmd = decode_cmd();//使用するコマンド構造体の宣言
			command cmd_pre = cmd;//前回のコマンドを保存

			// ------------------------------------------------------------------
			// 自動移動用の目標座標（必要に応じて数値を調整してください）
			// ※ z軸のPID制御は削除したため、xとthのみ指定します
			// ------------------------------------------------------------------
			const float pos_A_x = -30.2f, pos_A_th = 112.0f;   // 上（hat_ver == 2）入力時の目標位置
			const float pos_B_x = 4.39f, pos_B_th = 160.05f;  // 右（hat_hor == 2）入力時の目標位置
			const float pos_Xl_x = 26.35f, pos_Xl_th = 42.69f; // 下（hat_ver == 0）入力時の目標位置
			const float pos_Xr_x = -26.35f, pos_Xr_th = 181.31f; // 下（hat_ver == 0）入力時の目標位置
			const float pos_Y_x = -4.29f, pos_Y_th = 63.95f;// 左（hat_hor == 0）入力時の目標位置

			//左右の選択
			HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
			for(;;){
				command cmd = decode_cmd();
				if(!cmd_pre.B && cmd.B){
					HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
					RorB = 0;
					break;
				}
				else if(!cmd_pre.X && cmd.X){
					HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
					RorB = 1;
					break;
				}
				cmd_pre = cmd;
				osDelay(10);
			}

			//使用する機器の宣言と初期化
			kondo mainarm;
			init_kondo(&mainarm, mainarm_, 7);
			osDelay(10);

			dcmotor motor_x, motor_z, motor_th, minimotor, waterpomp;
			init_x(&motor_x, motor_x_, 1,0);
			osDelay(10);
			init_z(&motor_z, motor_z_, 2,0);
			osDelay(10);
			init_th(&motor_th, motor_th_, 2,1);
			osDelay(10);
			init_z(&minimotor, minimotor_, 3,0);
			osDelay(10);
			init_z(&waterpomp, waterpomp_, 3,1);
			osDelay(10);

			isStructInited_main = true;//メインでstructInit完了
			for(;;){//サブでstructinitされるまで待つ
				if(isStructInited_sub) break;
				osDelay(10);
			}

			// 全デバイス & CAN初期化
			ModularCANLib_AllDevice_And_CANSystem_Init();

			// 起動待ち
			ModularCANLib_WaitForConnect();

			isSystemInited = true;

			//メインアームを起動し、0度にする
			start_kondo(mainarm);
			Kondo_SetTarget_With_Time(mainarm.dev_info_kondo, 0.0f, 1.0f);

			//x直動を起動し、キャリブレーションを行う
			MCMD_init(motor_x.dev_info);
			MCMD_Calib(motor_x.dev_info);

			//z直動を起動し、キャリブレーションを行う
			MCMD_init(motor_z.dev_info);
			MCMD_Calib(motor_z.dev_info);

			//th直動を起動し、キャリブレーションを行う
			MCMD_init(motor_th.dev_info);
			MCMD_Calib(motor_th.dev_info);

			cmd_pre = decode_cmd();
			const float d_max_x = 0.03f, d_max_th = 0.028f, d_max_z = 0.03f;

			// x軸およびth軸のPID構造体初期化 {Kp, Ki, Kd, Target(r), Clip, Integral, Pos_pre}
			PID pid_x  = {0.08f, 0.0f, 0.0f, 0.0f, 0.03f,  0.0f, 0.0f};
			PID pid_th = {0.03f, 0.0f, 0.0f, 0.0f, 0.03f, 0.0f, 0.0f};

			bool isPID_main = false; // PID制御中かどうかのフラグ*/
			int c = 0;

		  /* Infinite loop */
		  for(;;)
		  {
			  cmd = decode_cmd();//コマンド更新

			  // メインアーム操作
			  if(!cmd_pre.START && cmd.START) {
				  spin_mainarm(mainarm, &c);
			  }
			  else if (!cmd_pre.BACK && cmd.BACK) {
				  release_mainarm(mainarm, &c);
			  }

			  // ジョイスティック入力値の取得 (-1.0 ～ 1.0)
			  float d_x  = ((float)cmd.right_hor / 255.0f) * 2.0f - 1.0f;
			  float d_th = ((float)cmd.left_hor  / 255.0f) * 2.0f - 1.0f;
			  float d_z  = ((float)cmd.right_ver  / 255.0f) * 2.0f - 1.0f;

			  // ------------------------------------------------------------------
			  // z軸のデッドゾーン処理（下方向の閾値を大きく設定）
			  // ------------------------------------------------------------------
			  const float deadzone_up   = 0.10f; // 上方向（正）の不感帯
			  const float deadzone_down = 0.30f; // 下方向（負）の不感帯（広く設定）

			  if (d_z < 0.0f) {
				  // 下方向に入力されている場合
				  if (d_z > -deadzone_down) {
					  d_z = 0.0f; // 広めのデッドゾーン領域内なら0にする
				  }
			  } else {
				  // 上方向に入力されている場合
				  if (d_z < deadzone_up) {
					  d_z = 0.0f; // 通常のデッドゾーン領域内なら0にする
				  }
			  }

			  if(cmd.RB) DUTYmove(&motor_x,  d_x  * d_max_x * 1.5f);//RBで早く動く
			  else DUTYmove(&motor_x,  d_x  * d_max_x);
			  DUTYmove(&motor_th, d_th * d_max_th);
			  //}

			  // z軸の駆動制御（常にDUTY手動制御のみ）
			  DUTYmove(&motor_z, d_z * d_max_z);

			  cmd_pre = cmd; // 前回のコマンドを更新
			  osDelay(1);
		  }
  /* USER CODE END StartMainTask */
}

/* USER CODE BEGIN Header_StartSubTask */
/**
* @brief Function implementing the SubTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSubTask */
void StartSubTask(void const * argument)
{
  /* USER CODE BEGIN StartSubTask */
	const float vertical = 114.0f;
	const float box_pos[5] = {0.8475, 9.515f, 18.11f, 26.80f, 27.78f};//一番手前が原点で、奥がプラス（箱１、箱２、箱３、箱４、一番奥）
	const float buf = 0.2f;
	for(;;){//赤か青が選ばれるまで待つ
		if(RorB!=-1) break;
		osDelay(10);
	}
	for(;;){//メインでストラクトinitされるまで待つ
		if(isStructInited_main) break;
		osDelay(10);
	}
	command cmd = decode_cmd();//使用するコマンド構造体
	command cmd_pre = cmd;//前回のコマンドを保存
	//使用する機器の宣言と初期化
	kondo subarm, subdoor;
	init_kondo(&subarm, subarm_, 6);
	osDelay(10);
	init_kondo(&subdoor, subdoor_, 2);
	osDelay(10);

	dcmotor motor_y;
	init_y(&motor_y, motor_y_, 1,1);
	isStructInited_sub = true;//structinit完了
	for(;;){//systeminitされるまで待つ
		if(isSystemInited) break;
		osDelay(10);
	}

	//サブコンドウを起動し、それぞれ20度（鉛直）, 3度(扉がほぼ閉まってる)にする
	start_kondo(subdoor);
	Kondo_SetTarget_With_Time(subdoor.dev_info_kondo, 0.0f, 0.5f);
	osDelay(1000);

	start_kondo(subarm);
	Kondo_SetTarget_With_Time(subarm.dev_info_kondo, vertical, 2.0f);
	osDelay(3000);
	//y直動を起動し、キャリブレーションを行う
	MCMD_init(motor_y.dev_info);
	MCMD_Calib(motor_y.dev_info);  // キャリブレーションを行う
	//MCMD_Wait_For_Calib(motor_y.dev_info);  // キャリブレーションが終わるまでまつ

  /* Infinite loop */
	cmd_pre = decode_cmd();//前回のコマンド
	PID pid_y = {0.08f,0.0f,0.0f,0.0f, 0.03f, 0.0f, 0.0f};//PID構造体
	const float d = 0.03f;//yの手動制御のduty比

	bool isPID = false, islied = false;//pid制御をするかどうかのフラグと、倒れているかを制御するフラグ
  for(;;)
  {
	  cmd = decode_cmd();//コマンド更新
	  //手動で動かす
	  if(cmd.sec_A && !islied) {
		  DUTYmove(&motor_y, -d);//sec_Aが入力されていたら後退
		  isPID = false;//PID制御オフ
	  }
	  else if(!cmd.sec_A && cmd_pre.sec_A) {
		  DUTYmove(&motor_y, 0.0f);//sec_Aが離されたら止まる
		  isPID = false;//PID制御オフ
	  }
	  else if(cmd.sec_Y && !islied) {
		  DUTYmove(&motor_y, d);//sec_Yが入力されていたら前進
		  isPID = false;//PID制御オフ
	  }
	  else if(!cmd.sec_Y && cmd_pre.sec_Y) {
		  DUTYmove(&motor_y, 0.0f);//sec_Yが離されたら止まる
		  isPID = false;//PID制御オフ
	  }
	  else if(!cmd_pre.sec_BACK && cmd.sec_BACK){//自動制御の後退入力
		  float pos = motor_pos(&motor_y);//現在地を取得
		  isPID = true;//PID制御をon
		  if(islied){//倒れていたら立てる
			  Kondo_SetTarget_With_Time(subdoor.dev_info_kondo, 0.0f, 0.5f);//扉閉じる
			  osDelay(1000);
			  Kondo_SetTarget_With_Time(subarm.dev_info_kondo, vertical, 2.0f);//立てる
			  osDelay(3000);
			  islied = false;
		  }
		  else if(pos < box_pos[0] + buf){//一番手前の箱まできたらPIDoff
			  isPID = false;
			  DUTYmove(&motor_y, 0.0f);//モーター停止
		  }
		  else if(pos < box_pos[1]+buf){//箱0に向かう
			  pid_y.r = box_pos[0];
		  }
		  else if(pos<box_pos[2] + buf){//箱1に向かう
			  pid_y.r = box_pos[1];
		  }
		  else if(pos < box_pos[3] + buf){//箱2に向かう
			  pid_y.r = box_pos[2];
		  }
		  else {//箱3に向かう
			  pid_y.r = box_pos[3];
		  }
	  }
	  else if(!cmd_pre.sec_LB && cmd.sec_LB && !islied){//自動制御の前進入力
		  float pos = motor_pos(&motor_y);//現在地を取得
		  isPID = true;//PID制御をon
		  if(pos > box_pos[4] - buf){//一番奥にいるなら倒す
			  Kondo_SetTarget_With_Time(subdoor.dev_info_kondo, 0.0f, 0.5f);//扉閉じる
			  osDelay(1000);
			  if(RorB == Blue) Kondo_SetTarget_With_Time(subarm.dev_info_kondo, vertical + 80.0f, 4.0f);//倒す
			  else Kondo_SetTarget_With_Time(subarm.dev_info_kondo, vertical - 80.0f, 4.0f);
			  osDelay(5000);
			  islied = true;
			  isPID = false;
		  }
		  else if(pos > box_pos[2] - buf){
			  pid_y.r = box_pos[3];
		  }
		  else if(pos > box_pos[1] - buf){
			  pid_y.r = box_pos[2];
		  }
		  else if(pos > box_pos[0] - buf){
			  pid_y.r = box_pos[1];
		  }
		  else {
			  pid_y.r = box_pos[0];
		  }
	  }
	  if(!cmd_pre.sec_B && cmd.sec_B){//sec_Bが押されたら
		  ModularCANLib_CANDynamixel_Kondo_Feedback_Type kondo_fb = Get_Kondo_Feedback(subdoor.dev_info_kondo);//フィードバックを取得
		  float r = kondo_fb.value;
		  if(r < 60.0f) Kondo_SetTarget_With_Time(subdoor.dev_info_kondo, 60.0f, 0.2f);//一気にドア開く
		  else Kondo_SetTarget_With_Time(subdoor.dev_info_kondo, 120.0f, 0.2f);//一気にドア開く
		  osDelay(300);//0.4secまつ
	  }
	  else if(!cmd_pre.sec_X && cmd.sec_X){//sec_Xが押されたら
		  ModularCANLib_CANDynamixel_Kondo_Feedback_Type kondo_fb = Get_Kondo_Feedback(subdoor.dev_info_kondo);//フィードバックを取得
		  float r = kondo_fb.value;
		  //ちょっと戻す
		  if(r<10) Kondo_SetTarget_With_Time(subdoor.dev_info_kondo, 0.0f, 0.1f);
		  else if (r < 61)Kondo_SetTarget_With_Time(subdoor.dev_info_kondo, r-10.0f, 0.1f);
		  else Kondo_SetTarget_With_Time(subdoor.dev_info_kondo, 60.0f, 0.5f);
		  //osDelay(300);//0.3secまつ
	  }
	  if(isPID) {
		  PIDmove(&motor_y, &pid_y);
	  }
	  cmd_pre = cmd;
	  osDelay(10);
  }
  /* USER CODE END StartSubTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

