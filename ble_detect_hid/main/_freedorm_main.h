#ifndef _FREEDORM_MAIN_H_
#define _FREEDORM_MAIN_H_

typedef enum
{
    FREEDORM_STATE_DEFAULT,              // 默认状态
    FREEDORM_STATE_FIRST_POWER_ON,       // 第一次上电激活
    FREEDORM_STATE_RESET_CONFIRM,        // 确定恢复出厂设置
    FREEDORM_STATE_RESETTING,            // 恢复出厂设置中
    FREEDORM_STATE_SINGLE_OPEN,          // 单次开门
    FREEDORM_STATE_SINGLE_RESTORE,       // 单次恢复正常
    FREEDORM_STATE_ALWAYS_OPEN,          // 常开模式
    FREEDORM_STATE_LOCK,                 // 锁门状态
    FREEDORM_STATE_POWER_ON_ANIMATION,   // 上电动画
    FREEDORM_STATE_BLUETOOTH_PAIRING,    // 蓝牙配对开始
    FREEDORM_STATE_REMOTE_UNLOCK_REQUEST // 远程开门请求
} freedorm_state_t;

#endif // _FREEDORM_MAIN_H_
