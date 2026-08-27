/**
 * @file referee_protocol.h
 * @author anchengc
 * @brief 裁判系统协议定义
 * @version 0.1
 * @date 2026-06-11 0.1 初版
 * @date 2026-08-16 0.2 裁判系统通信协议v2.0.0更新
 *
 * @copyright SZTU-HJ (c) 2026
 *
 */

#pragma once

/* Includes ------------------------------------------------------------------*/
#include <cstdint>

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 裁判系统数据帧头结构体
 *
 */
struct __attribute__((packed)) FrameHeader
{
    uint8_t sof;
    uint16_t data_length;
    uint8_t seq;
    uint8_t crc8;
};

/**
 * @brief 裁判系统命令码类型
 *
 */
enum RefereeCmdId
{
    // clang-format off
    REFEREE_CMD_ID_GAME_STATUS                           = 0x0001, // len: 11, 比赛状态数据, 1Hz, 服务器->全体机器人, 常规链路
    REFEREE_CMD_ID_GAME_RESULT                           = 0x0002, // len: 1, 比赛结果数据, 比赛结束触发, 服务器->全体机器人, 常规链路
    REFEREE_CMD_ID_GAME_ROBOT_HP                         = 0x0003, // len: 20, 机器人血量数据, 3Hz, 服务器->全体机器人, 常规链路
    REFEREE_CMD_ID_EVENT_DATA                            = 0x0101, // len: 4, 场地事件数据, 1Hz, 服务器->己方全体机器人, 常规链路
    REFEREE_CMD_ID_REFEREE_WARNING                       = 0x0104, // len: 3, 裁判警告数据, 判罚/判负触发且其余时间1Hz, 服务器->被判罚方全体机器人, 常规链路
    REFEREE_CMD_ID_DART_LAUNCHING_STATUS                 = 0x0105, // len: 3, 飞镖发射相关数据, 1Hz, 服务器->己方全体机器人, 常规链路
    REFEREE_CMD_ID_GAME_ROBOT_STATE                      = 0x0201, // len: 17, 机器人性能体系数据, 10Hz, 主控模块->对应机器人, 常规链路
    REFEREE_CMD_ID_POWER_HEAT_DATA                       = 0x0202, // len: 14, 实时底盘缓冲能量和射击热量数据, 10Hz, 主控模块->对应机器人, 常规链路
    REFEREE_CMD_ID_GAME_ROBOT_POS                        = 0x0203, // len: 16, 机器人位置数据, 1Hz, 主控模块->对应机器人, 常规链路
    REFEREE_CMD_ID_BUFF                                  = 0x0204, // len: 8, 机器人增益和底盘能量数据, 3Hz, 服务器->对应机器人, 常规链路
    REFEREE_CMD_ID_ROBOT_HURT                            = 0x0206, // len: 1, 伤害状态数据, 伤害发生后发送, 主控模块->对应机器人, 常规链路
    REFEREE_CMD_ID_SHOOT_DATA                            = 0x0207, // len: 7, 实时射击数据, 弹丸发射后发送, 主控模块->对应机器人, 常规链路
    REFEREE_CMD_ID_PROJECTILE_ALLOWANCE                  = 0x0208, // len: 8, 允许发弹量与剩余金币数, 10Hz, 服务器->己方英雄/步兵/哨兵/空中机器人, 常规链路
    REFEREE_CMD_ID_ROBOT_RFID                            = 0x0209, // len: 5, 机器人RFID模块状态, 3Hz, 服务器->己方装有RFID模块的机器人, 常规链路
    REFEREE_CMD_ID_ROBOT_DART_COMMAND                    = 0x020A, // len: 6, 飞镖选手端指令数据, 3Hz, 服务器->己方飞镖机器人, 常规链路
    REFEREE_CMD_ID_GROUND_ROBOT_LOCATION                 = 0x020B, // len: 40, 地面机器人位置数据, 1Hz, 服务器->己方哨兵机器人, 常规链路
    REFEREE_CMD_ID_RADAR_TRACKING_PROGRESS               = 0x020C, // len: 2, 雷达标记进度数据, 1Hz, 服务器->己方雷达机器人, 常规链路
    REFEREE_CMD_ID_SENTRY_DECISION_SYNC                  = 0x020D, // len: 14, 哨兵自主决策信息同步, 1Hz, 服务器->己方哨兵机器人, 常规链路
    REFEREE_CMD_ID_RADAR_DECISION_SYNC                   = 0x020E, // len: 1, 雷达自主决策信息同步, 1Hz, 服务器->己方雷达机器人, 常规链路
    REFEREE_CMD_ID_ROBOT_INTERACTIVE_DATA                = 0x0301, // len: 118, 机器人交互数据, 触发发送且上限30Hz, 常规链路
    REFEREE_CMD_ID_ROBOT_RECEIVE_CUSTOM_CONTROLLER_DATA  = 0x0302, // len: 30, 自定义控制器与机器人交互数据, 触发发送且上限30Hz, 自定义控制器->选手端图传连接的机器人, 图传链路
    REFEREE_CMD_ID_CLIENT_MINIMAP_INTERACTIVE_DATA       = 0x0303, // len: 15, 选手端小地图交互数据, 选手端触发, 选手端点击->服务器->发送方选择的己方机器人, 常规链路
    REFEREE_CMD_ID_CLIENT_MINIMAP_RECEIVE_RADAR_DATA     = 0x0305, // len: 48, 选手端小地图接收雷达数据, 上限5Hz, 雷达->服务器->己方所有选手端, 常规链路
    REFEREE_CMD_ID_CUSTOM_CONTROLLER_CLIENT_DATA         = 0x0306, // len: 8, 自定义控制器与选手端交互数据, 触发发送且上限30Hz, 自定义控制器->选手端
    REFEREE_CMD_ID_CLIENT_MINIMAP_RECEIVE_PATH_DATA      = 0x0307, // len: 103, 选手端小地图接收路径数据, 上限1Hz, 哨兵/半自动控制机器人->对应操作手选手端, 常规链路
    REFEREE_CMD_ID_CLIENT_MINIMAP_RECEIVE_ROBOT_DATA     = 0x0308, // len: 34, 选手端小地图接收机器人数据, 上限3Hz, 己方机器人->己方选手端, 常规链路
    REFEREE_CMD_ID_CUSTOM_CONTROLLER_RECEIVE_ROBOT_DATA  = 0x0309, // len: 30, 自定义控制器接收机器人数据, 上限10Hz, 己方机器人->对应操作手选手端连接的自定义控制器, 图传链路
    REFEREE_CMD_ID_CUSTOM_CLIENT_RECEIVE_ROBOT_DATA      = 0x0310, // len: 300, 机器人发送给自定义客户端的数据, 上限50Hz, 己方机器人->图传链路->对应操作手选手端连接的自定义客户端, 图传链路
    REFEREE_CMD_ID_ROBOT_RECEIVE_CUSTOM_CLIENT_DATA      = 0x0311, // len: 30, 自定义客户端发送给机器人的自定义指令, 上限75Hz, 自定义客户端->图传链路->己方机器人, 图传链路
    // clang-format on
};
// 0x0001 比赛状态数据
struct __attribute__((packed)) RefereeRxDataGameStatus
{
    // bit 0-3: 比赛类型
    //       1: RoboMaster机甲大师超级对抗赛
    //       2: RoboMaster机甲大师高校单项赛
    //       3: ICRA RoboMaster高校人工智能挑战赛
    //       4: RoboMaster机甲大师高校联盟赛3V3对抗
    //       5: RoboMaster机甲大师高校联盟赛步兵对抗
    uint8_t game_type : 4;

    // bit 4-7: 当前比赛阶段
    //       0: 未开始比赛
    //       1: 准备阶段
    //       2: 十五秒裁判系统自检阶段
    //       3: 五秒倒计时
    //       4: 比赛中
    //       5: 比赛结算中
    uint8_t game_progress : 4;

    // 当前阶段剩余时间，单位：s
    uint16_t stage_remain_time;
    // UNIX时间，当机器人正确连接到裁判系统的NTP服务器后生效
    uint64_t sync_time_stamp;
};

// 0x0002 比赛结果数据
struct __attribute__((packed)) RefereeRxDataGameResult
{
    // 0: 平局, 1: 红方胜利, 2: 蓝方胜利
    uint8_t winner;
};

// 0x0003 比赛机器人血量数据
struct __attribute__((packed)) RefereeRxDataGameRobotHp
{
    // 己方 1 英雄机器人血量
    uint16_t ally_1_robot_hp;
    // 己方 2 工程机器人血量
    uint16_t ally_2_robot_hp;
    // 己方 3 步兵机器人血量
    uint16_t ally_3_robot_hp;
    // 己方 4 步兵机器人血量
    uint16_t ally_4_robot_hp;
    // 己方全队总伤害与对方全队总伤害之差
    int16_t damage_difference;
    // 己方 7 哨兵机器人血量
    uint16_t ally_7_robot_hp;
    // 己方前哨站血量
    uint16_t ally_outpost_hp;
    // 己方基地血量
    uint16_t ally_base_hp;
    // 对方前哨站血量
    uint16_t opponent_outpost_HP;
    // 对方基地血量
    uint16_t opponent_base_HP;
};

// 0x0101 机器人事件数据
struct __attribute__((packed)) RefereeRxDataEventData
{
    // 0:未占领/未激活
    // 1:已占领/已激活
    // bit 0：己方补给区的占领状态，1为已占领
    // bit 1：保留位
    // bit 2：己方补给区的占领状态，1 为已占领（仅 RMUL 适用）
    uint32_t supply_status : 3;

    // bit 3-4：己方小能量机关的激活状态，0 为未激活，1 为已激活，2 为正在激活
    // bit 5-6：己方大能量机关的激活状态，0 为未激活，1 为已激活，2 为正在激活
    uint32_t power_rune_status : 4;

    // bit 7-8：己方中央高地的占领状态，1 为被己方占领，2 为被对方占领
    uint32_t central_highground_status : 2;

    // bit 9-10：己方梯形高地的占领状态，1 为已占领
    uint32_t trapezoidal_highground_status : 2;

    // bit 11-19：对方飞镖最后一次击中己方前哨站或基地的时间（0-420，开局默认为 0）
    uint32_t last_hit_time : 9;

    // bit 20-22：对方飞镖最后一次击中己方前哨站或基地的具体目标，开局默认为 0
    // 1 为击中前哨站，2 为击中基地固定目标，3 为击中基地随机固定目标，4 为击中基地随机移动目标
    // 5 为击中基地末端移动目标
    uint32_t last_hit_target : 3;

    // bit 23-24：中心增益点的占领状态，0 为未被占领，1 为被己方占领，2 为被对方占领
    // 3 为被双方占领。（仅 RMUL 适用）
    uint32_t center_buff_status : 2;

    // bit 25-26：己方堡垒增益点的占领状态，0 为未被占领，1 为被己方占领，2 为被对方占领
    // 3 为被双方占领。（仅 RMUL 适用）
    uint32_t fortress_buff_status : 2;

    // bit 27-28：己方前哨站增益点的占领状态，0 为未被占领，1 为被己方占领，2 为被对方占领
    uint32_t outpost_buff_status : 2;

    // bit 29：己方基地增益点的占领状态，1 为已占领
    uint32_t base_buff_status : 1;

    // bit 30-31：保留位
    uint32_t reserved : 2;
};

// 0x0104 裁判警告数据
struct __attribute__((packed)) RefereeRxDataRefereeWarning
{
    // 己方最后一次受到判罚的等级
    uint8_t level;
    // 己方最后一次受到判罚的违规机器人 ID
    uint8_t offending_robot_id;
    // 己方最后一次受到判罚的违规次数
    uint8_t count;
};

// 0x0105 飞镖发射相关数据
struct __attribute__((packed)) RefereeRxDataDartInfo
{
    // 己方飞镖发射站剩余发射时间，单位：s
    uint8_t dart_remaining_time;

    // bit 0-2：己方飞镖最后一次击中目标
    //       0：开局默认值
    //       1：前哨站
    //       2：基地固定目标
    //       3：基地随机固定目标
    //       4：基地随机移动目标
    //       5：基地末端移动目标
    uint16_t last_hit_target : 3;

    // bit 3-5：对方最近被击中目标的累计次数，开局默认值为 0，至多为 4
    uint16_t total_hit_count : 3;

    // bit 6-8：当前飞镖选择的击打目标，开局默认值为 0
    //       1：基地固定目标
    //       2：基地随机固定目标
    //       3：基地随机移动目标
    //       4：基地末端移动目标
    uint16_t current_selected_target : 3;

    // bit 9-15：保留
    uint16_t reserved : 7;
};

// 0x0201 机器人性能体系数据
struct __attribute__((packed)) RefereeRxDataRobotState
{
    // 机器人ID
    uint8_t robot_id;
    // 机器人等级
    uint8_t robot_level;
    // 当前血量
    uint16_t current_hp;
    // 血量上限
    uint16_t maximum_hp;
    // 发射机构热量每秒冷却值
    uint16_t shooter_barrel_cooling_value;
    // 发射机构热量上限
    uint16_t shooter_barrel_heat_limit;
    // 底盘功率上限，单位：W
    uint16_t chassis_power_limit;
    // 射击初速度上限，单位：m/s
    float bullet_speed_limit;

    // gimbal口输出: 0为无输出, 1为24V输出
    uint8_t power_management_gimbal_output : 1;
    // chassis口输出: 0为无输出, 1为24V输出
    uint8_t power_management_chassis_output : 1;
    // shooter口输出: 0为无输出, 1为24V输出
    uint8_t power_management_shooter_output : 1;
};

// 0x0202 实时底盘缓冲能量和射击热量数据
struct __attribute__((packed)) RefereeRxDataPowerHeatData
{
    uint16_t reserved_1;
    uint16_t reserved_2;
    float reserved_3;

    // 缓冲能量，单位：J
    uint16_t buffer_energy;
    // 17mm 发射机构的射击热量
    uint16_t shooter_17mm_barrel_heat;
    // 42mm 发射机构的射击热量
    uint16_t shooter_42mm_barrel_heat;
};

// 0x0203 机器人位置数据
struct __attribute__((packed)) RefereeRxDataRobotPos
{
    // 本机器人位置 x 坐标，单位：m
    float x;
    // 本机器人位置 y 坐标，单位：m
    float y;
    // 本机器人测速模块朝向，单位：度，正北为 0 度
    float angle;
};

// 0x0204 机器人增益数据
struct __attribute__((packed)) RefereeRxDataBuff
{
    // 机器人回血增益（百分比，值为 10 表示每秒恢复血量上限的 10%）
    uint8_t recovery_buff;
    // 机器人射击热量冷却增益具体值（直接值，值为 x 表示热量冷却增加 x/s）
    uint16_t cooling_buff;
    // 机器人防御增益（百分比，值为 50 表示 50%防御增益）
    uint8_t defense_buff;
    // 机器人负防御增益（百分比，值为 30 表示-30%防御增益）
    uint8_t vulnerability_buff;
    // 机器人攻击增益（百分比，值为 50 表示 50%攻击增益）
    uint16_t attack_buff;

    // bit 0-6：机器人剩余能量值反馈，以16进制标识机器人剩余能量值比例。机器人初始能量视为100%
    // bit 0：在剩余能量≥125%时为 1，其余情况为 0
    // bit 1：在剩余能量≥100%时为 1，其余情况为 0
    // bit 2：在剩余能量≥50%时为 1，其余情况为 0
    // bit 3：在剩余能量≥30%时为 1，其余情况为 0
    // bit 4：在剩余能量≥15%时为 1，其余情况为 0
    // bit 5：在剩余能量≥5%时为 1，其余情况为 0
    // bit 6：在剩余能量≥1%时为 1，其余情况为 0
    uint8_t remaining_energy;
};

// 0x0206 伤害状态数据
struct __attribute__((packed)) RefereeRxDataHurtData
{
    // bit 0-3：当扣血原因为装甲模块被弹丸攻击、受撞击、离线或测速模块离线时，
    // 该 4 bit 组成的数值为装甲模块或测速模块的 id 编号；
    // 当其他原因导致扣血时，该数值为 0
    uint8_t armor_id : 4;

    // bit 4-7：血量变化类型
    //       0：装甲模块被弹丸攻击导致扣血
    //       1：装甲模块或超级电容管理模块离线导致扣血
    //       2：射速超限导致扣血
    //       3：枪口热量超限导致扣血
    //       4：底盘功率超限导致扣血
    //       5：装甲模块受到撞击导致扣血
    //       6：测速模块离线导致扣血
    uint8_t hp_deduction_reason : 4;
};

// 0x0207 实时射击数据
struct __attribute__((packed)) RefereeRxDataShootData
{
    // bit1：17mm弹丸, bit2：42mm弹丸
    uint8_t projectile_type;
    // 1: 17mm发射机构, 2: 保留位, 3: 42mm发射机构
    uint8_t shooter_number;
    // 射频，单位: Hz
    uint8_t launching_frequency;
    // 弹丸初速度，单位: m/s
    float projectile_speed;
};

// 0x0208 允许发弹量数据
struct __attribute__((packed)) RefereeRxDataProjectileAllowance
{
    // 17mm 弹丸允许发弹量
    uint16_t projectile_allowance_17mm;
    // 42mm 弹丸允许发弹量
    uint16_t projectile_allowance_42mm;
    // 剩余金币数量
    uint16_t remaining_gold_coin;

    // 堡垒增益点提供的储备 17mm 弹丸允许发弹量；
    // 该值与机器人是否实际占领堡垒无关
    uint16_t projectile_allowance_fortress;
};

// 0x0209 机器人RFID模块状态
struct __attribute__((packed)) RefereeRxDataRfidStatus
{
    // bit 0：己方基地增益点
    uint32_t ally_base : 1;
    // bit 1：己方中央高地增益点
    uint32_t ally_central : 1;
    // bit 2：对方中央高地增益点
    uint32_t opponent_central : 1;
    // bit 3：己方梯形高地增益点
    uint32_t ally_trapezoid : 1;
    // bit 4：对方梯形高地增益点
    uint32_t opponent_trapezoid : 1;
    // bit 5：己方地形跨越增益点（飞坡前，靠近己方一侧）
    uint32_t ally_flying_slope_f : 1;
    // bit 6：己方地形跨越增益点（飞坡后，靠近己方一侧）
    uint32_t ally_flying_slope_b : 1;
    // bit 7：对方地形跨越增益点（飞坡前，靠近对方一侧）
    uint32_t opponent_flying_slope_f : 1;
    // bit 8：对方地形跨越增益点（飞坡后，靠近对方一侧）
    uint32_t opponent_flying_slope_b : 1;
    // bit 9：己方地形跨越增益点（中央高地下方）
    uint32_t ally_ch_down : 1;
    // bit 10：己方地形跨越增益点（中央高地上方）
    uint32_t ally_ch_up : 1;
    // bit 11：对方地形跨越增益点（中央高地下方）
    uint32_t opponent_ch_down : 1;
    // bit 12：对方地形跨越增益点（中央高地上方）
    uint32_t opponent_ch_up : 1;
    // bit 13：己方地形跨越增益点（公路下方）
    uint32_t ally_road_down : 1;
    // bit 14：己方地形跨越增益点（公路上方）
    uint32_t ally_road_up : 1;
    // bit 15：对方地形跨越增益点（公路下方）
    uint32_t opponent_road_down : 1;
    // bit 16：对方地形跨越增益点（公路上方）
    uint32_t opponent_road_up : 1;
    // bit 17：己方堡垒增益点
    uint32_t ally_fort : 1;
    // bit 18：己方前哨站增益点
    uint32_t ally_outpost : 1;
    // bit 19：己方与资源区不重叠的补给区 / RMUL 补给区
    uint32_t ally_supply_no_resource : 1;
    // bit 20：己方与资源区重叠的补给区
    uint32_t ally_supply_zone : 1;
    // bit 21：己方装配增益点
    uint32_t ally_assembly_buff : 1;
    // bit 22：对方装配增益点
    uint32_t opponent_assembly_buff : 1;
    // bit 23：中心增益点（仅 RMUL 适用）
    uint32_t center_point : 1;
    // bit 24：对方堡垒增益点
    uint32_t opponent_fort : 1;
    // bit 25：对方前哨站增益点
    uint32_t opponent_outpost : 1;
    // bit 26：己方地形跨越增益点（隧道）（靠近己方一侧公路区下方）
    uint32_t ally_tunnel_road_down : 1;
    // bit 27：己方地形跨越增益点（隧道）（靠近己方一侧公路区中间）
    uint32_t ally_tunnel_road_mid : 1;
    // bit 28：己方地形跨越增益点（隧道）（靠近己方一侧公路区上方）
    uint32_t ally_tunnel_road_up : 1;
    // bit 29：己方地形跨越增益点（隧道）（靠近己方梯形高地较低处）
    uint32_t ally_tunnel_trapezoid_low : 1;
    // bit 30：己方地形跨越增益点（隧道）（靠近己方梯形高地较中间处）
    uint32_t ally_tunnel_trapezoid_mid : 1;
    // bit 31：己方地形跨越增益点（隧道）（靠近己方梯形高地较高处）
    uint32_t ally_tunnel_trapezoid_high : 1;

    // bit 0：对方地形跨越增益点（隧道）（靠近对方一侧公路区下方）
    uint8_t opponent_tunnel_road_down : 1;
    // bit 1：对方地形跨越增益点（隧道）（靠近对方一侧公路区中间）
    uint8_t opponent_tunnel_road_mid : 1;
    // bit 2：对方地形跨越增益点（隧道）（靠近对方一侧公路区上方）
    uint8_t opponent_tunnel_road_up : 1;
    // bit 3：对方地形跨越增益点（隧道）（靠近对方梯形高地较低处）
    uint8_t opponent_tunnel_trapezoid_low : 1;
    // bit 4：对方地形跨越增益点（隧道）（靠近对方梯形高地较中间）
    uint8_t opponent_tunnel_trapezoid_mid : 1;
    // bit 5：对方地形跨越增益点（隧道）（靠近对方梯形高地较高处）
    uint8_t opponent_tunnel_trapezoid_high : 1;
    // bit 6-7
    uint8_t reserved : 2;
};

// 0x020A 飞镖选手端指令数据
struct __attribute__((packed)) RefereeRxDataDartClientCmd
{
    // 当前飞镖发射站的状态： 1：关闭 2：正在开启或者关闭中 0：已经开启
    uint8_t dart_launch_opening_status;
    uint8_t reserved;

    // 切换击打目标时的比赛剩余时间，单位：秒，无/未切换动作，默认为 0。
    uint16_t target_change_time;
    // 最后一次操作手确定发射指令时的比赛剩余时间，单位：秒，初始值为0
    uint16_t latest_launch_cmd_time;
};

// 0x020B 地面机器人位置数据
struct __attribute__((packed)) RefereeRxDataGroundRobotPosition
{
    // 己方英雄机器人位置 x 轴坐标，单位：m
    float hero_x;
    // 己方英雄机器人位置 y 轴坐标，单位：m
    float hero_y;
    // 己方工程机器人位置 x 轴坐标，单位：m
    float engineer_x;
    // 己方工程机器人位置 y 轴坐标，单位：m
    float engineer_y;
    // 己方 3 号步兵机器人位置 x 轴坐标，单位：m
    float infantry_3_x;
    // 己方 3 号步兵机器人位置 y 轴坐标，单位：m
    float infantry_3_y;
    // 己方 4 号步兵机器人位置 x 轴坐标，单位：m
    float infantry_4_x;
    // 己方 4 号步兵机器人位置 y 轴坐标，单位：m
    float infantry_4_y;
    // 保留
    float reserved_1;
    // 保留
    float reserved_2;
};

// 0x020C 雷达标记进度数据
struct __attribute__((packed)) RefereeRxDataRadarMarkData
{
    // 对方机器人：在对应机器人被标记进度≥100 时发送 1，被标记进度<100 时发送 0。
    // 己方机器人：在对应机器人被标记进度≥50 时发送 1，被标记进度<50 时发送 0。
    // bit 0：对方 1号英雄机器人易伤情况
    uint16_t opponent_hero_1_vulnerable : 1;
    // bit 1：对方 2号工程机器人易伤情况
    uint16_t opponent_engineer_2_vulnerable : 1;
    // bit 2：对方 3号步兵机器人易伤情况
    uint16_t opponent_infantry_3_vulnerable : 1;
    // bit 3：对方 4号步兵机器人易伤情况
    uint16_t opponent_infantry_4_vulnerable : 1;
    // bit 4：对方空中机器人特殊标识情况
    uint16_t opponent_aerial_special : 1;
    // bit 5：对方哨兵机器人易伤情况
    uint16_t opponent_sentry_vulnerable : 1;
    // bit 6：己方 1号英雄机器人特殊标识情况
    uint16_t ally_hero_1_special : 1;
    // bit 7：己方 2号工程机器人特殊标识情况
    uint16_t ally_engineer_2_special : 1;
    // bit 8：己方 3号步兵机器人特殊标识情况
    uint16_t ally_infantry_3_special : 1;
    // bit 9：己方 4号步兵机器人特殊标识情况
    uint16_t ally_infantry_4_special : 1;
    // bit 10：己方空中机器人特殊标识情况
    uint16_t ally_aerial_special : 1;
    // bit 11：己方哨兵机器人特殊标识情况
    uint16_t ally_sentry_special : 1;
    // bit 12：对方空中机器人被雷达激光瞄准标识
    uint16_t opponent_aerial_radar_hit : 1;
    // bit 13：对方空中机器人被反制标识
    uint16_t opponent_aerial_suppressed : 1;
    // bit 14：己方空中机器人被雷达激光瞄准标识
    uint16_t ally_aerial_radar_hit : 1;
    // bit 15：己方空中机器人被反制标识
    uint16_t ally_aerial_suppressed : 1;
};

// 0x020D 哨兵自主决策信息同步
struct __attribute__((packed)) RefereeRxDataSentryInfo
{
    // bits 0-10：除远程兑换外，哨兵机器人成功兑换的允许发弹量，开局为0
    uint32_t allowed_fire_amount : 11;
    // bits 11-14：哨兵机器人成功远程兑换允许发弹量的次数，开局为0
    uint32_t remote_exchange_fire_count : 4;
    // bits 15-18 ：哨兵机器人成功远程兑换血量的次数，开局为0
    uint32_t remote_exchange_hp_count : 4;
    // bit 19 ：哨兵机器人当前是否可以确认免费复活，可以为1，否则为0
    uint32_t can_confirm_free_respawn : 1;
    // bit 20：哨兵机器人当前是否可以兑换立即复活，可以为1，否则为0
    uint32_t can_exchange_immediate_respawn : 1;
    // bits 21-30：哨兵机器人当前若兑换立即复活需要花费的金币数
    uint32_t immediate_respawn_cost : 10;
    // bit 31: 保留
    uint32_t reserved1 : 1;

    // bit 0：哨兵当前是否处于脱战状态，脱战为1，否则为0
    uint16_t out_of_combat : 1;
    // bit 1-11：队伍 17mm 允许发弹量的剩余可兑换数
    uint16_t ammo_exchange_allowance : 11;
    // bit 12-13：哨兵当前姿态：1进攻，2防御，3移动
    uint16_t sentry_pose : 2;
    // bit 14：己方能量机关是否能够进入正在激活状态，1为当前可激活
    uint16_t power_rune_status : 1;
    // bit 15：哨兵当前姿态是否为强化姿态
    uint16_t sentry_pose_boosted : 1;

    // bit 0-7：哨兵进攻姿态弱化前剩余可持续时长，单位：s
    uint64_t attack_pose_remaining_time : 8;
    // bit 8-15：哨兵防御姿态弱化前剩余可持续时长，单位：s
    uint64_t defense_pose_remaining_time : 8;
    // bit 16-23：哨兵移动姿态弱化前剩余可持续时长，单位：s
    uint64_t move_pose_remaining_time : 8;
    // bit 24-31：保留
    uint64_t reserved2 : 8;
    // bit 32-39：哨兵强化进攻姿态剩余可持续时长，单位：s
    uint64_t boosted_attack_pose_remaining_time : 8;
    // bit 40-47：哨兵强化防御姿态剩余可持续时长，单位：s
    uint64_t boosted_defense_pose_remaining_time : 8;
    // bit 48-55：哨兵强化移动姿态剩余可持续时长，单位：s
    uint64_t boosted_move_pose_remaining_time : 8;
    // bit 56-63：保留
    uint64_t reserved3 : 8;
};

// 0x020E 雷达自主决策信息同步
struct __attribute__((packed)) RefereeRxDataRadarInfo
{
    // bits 0-1：雷达拥有触发双倍易伤的机会（0~2），开局为 0，最大可达 2
    uint8_t double_vul_trigger_chance : 2;
    // bit 2：对方是否正在被触发双倍易伤 0：对方未被触发双倍易伤 1：对方正在被触发双倍易伤
    uint8_t opponent_in_double_vulnerability : 1;
    // bit 3-4：己方加密等级（即对方干扰波难度等级），开局为 1，最高为 3
    uint8_t ally_encryption_level : 2;
    // bit 5：当前是否可以修改密钥，1 为可修改
    uint8_t can_key_be_modified : 1;
    // bit 6-7：保留
    uint8_t reserved : 2;
};

// 0x0301 机器人交互数据
struct __attribute__((packed)) RefereeTxDataRobotInteractionData
{
    // 子内容id
    uint16_t sub_content_id;
    // 发送者的id，需校验
    uint16_t sender_id;
    // 接收者的id，需校验，除雷达外，仅支持发送机器人对应的选手端
    uint16_t receiver_id;
    // 最大为112
    uint8_t user_data[112];
};

// 0x0301 0x0100 选手端删除图层
struct __attribute__((packed)) RefereeTxDataInteractionLayerDelete
{
    // 0：空操作 1：删除图层 2：删除所有
    uint8_t delete_type;
    // 图层数：0~9
    uint8_t layer;
};

// 0x0301 0x0101 选手端绘制一个图形
struct __attribute__((packed)) RefereeTxDataInteractionFigure
{
    // 在图形删除、修改等操作中，作为索引
    uint8_t figure_name[3];

    // bit 0-2：图形操作 0：空操作 1：增加 2：修改 3：删除
    uint32_t operate_type : 3;
    // bit 3-5：图形类型 0：直线 1：矩形 2：正圆 3：椭圆 4：圆弧 5：浮点数 6：整型数 7：字符
    uint32_t figure_type : 3;
    // bit 6-9：图层数（0~9）
    uint32_t layer : 4;
    // bit 10-13：颜色 0：红/蓝（己方颜色） 1：黄色 2：绿色 3：橙色 4：紫红色 5：粉色 6：青色
    // 7：黑色 8：白色
    uint32_t color : 4;
    // bit 14-22：根据绘制的图形不同，含义不同
    uint32_t details_a : 9;
    // bit 23-31：根据绘制的图形不同，含义不同
    uint32_t details_b : 9;

    // bit 0-9：线宽，建议字体大小与线宽比例为 10：1
    uint32_t width : 10;
    // bit 10-20：起点/圆心 x 坐标
    uint32_t start_x : 11;
    // bit 21-31：起点/圆心 y 坐标
    uint32_t start_y : 11;

    // bit 0-9：根据绘制的图形不同，含义不同
    uint32_t details_c : 10;
    // bit 10-20：根据绘制的图形不同，含义不同
    uint32_t details_d : 11;
    // bit 21-31：根据绘制的图形不同，含义不同
    uint32_t details_e : 11;
};

// 0x0301 0x0102 选手端绘制两个图形
struct __attribute__((packed)) RefereeTxDataInteractionFigure2
{
    RefereeTxDataInteractionFigure interaction_figures[2];
};

// 0x0301 0x0103 选手端绘制五个图形
struct __attribute__((packed)) RefereeTxDataInteractionFigure5
{
    RefereeTxDataInteractionFigure interaction_figures[5];
};

// 0x0301 0x0104 选手端绘制七个图形
struct __attribute__((packed)) RefereeTxDataInteractionFigure7
{
    RefereeTxDataInteractionFigure interaction_figures[7];
};

// 0x0301 0x0110 选手端绘制字符图形
struct __attribute__((packed)) RefereeTxDataExtClientCustomCharacter
{
    RefereeTxDataInteractionFigure interaction_figure;
    uint8_t data[30];
};

// 0x0301 0x0120 哨兵自主决策指令
struct __attribute__((packed)) RefereeTxDataSentryCmd
{
    // bit 0：是否确认复活 0：否 1：是
    uint32_t confirm_respawn : 1;
    // bit 1：是否消耗金币兑换立即复活  0：否 1：是
    uint32_t immediate_respawn : 1;
    // bit 2-12：发弹量兑换值（递增有效）
    uint32_t projectile_allowance_exchange_amount : 11;
    // bit 13-16：远程兑换发弹量次数，开局为0，单调递增，每次加1
    uint32_t remote_exchange_projectile_allowance_count : 4;
    // bit 17-20：远程兑换血量次数，开局为0，单调递增，每次加1
    uint32_t remote_exchange_hp_count : 4;
    // bit 21-23：哨兵修改当前姿态指令，1 为进攻姿态，2 为防御姿态，3为移动姿态，
    // 4为强化进攻姿态，5为强化防御姿态，6为强化移动姿态。默认为3；修改此值即可改变哨兵姿态。
    uint32_t sentry_mode_cmd : 3;
    // bit 24：哨兵机器人是否确认使能量机关进入正在激活状态，1 为确认。默认为 0。
    uint32_t confirm_power_rune_activation : 1;
    // bit 25-31：保留
    uint32_t reserved : 7;
};

// 0x0301 0x0121 雷达自主决策指令
struct __attribute__((packed)) RefereeTxDataRadarCmd
{
    // 触发双倍易伤次数， 开局为0，单调递增，每次加1，最大为2
    uint8_t radar_cmd;

    // 密钥更新或验证指令
    uint8_t password_cmd;
    uint8_t password_1;
    uint8_t password_2;
    uint8_t password_3;
    uint8_t password_4;
    uint8_t password_5;
    uint8_t password_6;
};

// 0x0302 自定义控制器与机器人交互数据 图传链路
struct __attribute__((packed)) RefereeRxDataCustomRobotData
{
    uint8_t data[30];
};

// 0x0303 选手端小地图交互数据
struct __attribute__((packed)) RefereeRxDataMapCommandData
{
    // 目标位置 x 轴坐标，单位：m；当发送目标机器人 id 时，该值为 0
    float target_position_x;
    // 目标位置 y 轴坐标，单位：m；当发送目标机器人 id 时，该值为 0
    float target_position_y;
    // 云台手按下的键盘按键通用键值；无按键按下时为 0
    uint8_t cmd_keyboard;
    // 对方机器人 id；当发送坐标数据时，该值为 0
    uint8_t target_robot_id;
    // 信息来源 id
    uint16_t cmd_source;
};

// 0x0305 选手端小地图接收雷达数据
// x/y 超出边界时显示在对应边缘；x 和 y 均为 0 时，视为未发送此机器人坐标
struct __attribute__((packed)) RefereeTxDataMapRobotData
{
    // 对方英雄机器人 x 位置坐标，单位：cm
    uint16_t opponent_hero_position_x;
    // 对方英雄机器人 y 位置坐标，单位：cm
    uint16_t opponent_hero_position_y;
    // 对方工程机器人 x 位置坐标，单位：cm
    uint16_t opponent_engineer_position_x;
    // 对方工程机器人 y 位置坐标，单位：cm
    uint16_t opponent_engineer_position_y;
    // 对方 3 号步兵机器人 x 位置坐标，单位：cm
    uint16_t opponent_infantry_3_position_x;
    // 对方 3 号步兵机器人 y 位置坐标，单位：cm
    uint16_t opponent_infantry_3_position_y;
    // 对方 4 号步兵机器人 x 位置坐标，单位：cm
    uint16_t opponent_infantry_4_position_x;
    // 对方 4 号步兵机器人 y 位置坐标，单位：cm
    uint16_t opponent_infantry_4_position_y;
    // 对方 6 号空中机器人 x 位置坐标，单位：cm
    uint16_t opponent_aerial_position_x;
    // 对方 6 号空中机器人 y 位置坐标，单位：cm
    uint16_t opponent_aerial_position_y;
    // 对方哨兵机器人 x 位置坐标，单位：cm
    uint16_t opponent_sentry_position_x;
    // 对方哨兵机器人 y 位置坐标，单位：cm
    uint16_t opponent_sentry_position_y;
    // 己方英雄机器人 x 位置坐标，单位：cm
    uint16_t ally_hero_position_x;
    // 己方英雄机器人 y 位置坐标，单位：cm
    uint16_t ally_hero_position_y;
    // 己方工程机器人 x 位置坐标，单位：cm
    uint16_t ally_engineer_position_x;
    // 己方工程机器人 y 位置坐标，单位：cm
    uint16_t ally_engineer_position_y;
    // 己方 3 号步兵机器人 x 位置坐标，单位：cm
    uint16_t ally_infantry_3_position_x;
    // 己方 3 号步兵机器人 y 位置坐标，单位：cm
    uint16_t ally_infantry_3_position_y;
    // 己方 4 号步兵机器人 x 位置坐标，单位：cm
    uint16_t ally_infantry_4_position_x;
    // 己方 4 号步兵机器人 y 位置坐标，单位：cm
    uint16_t ally_infantry_4_position_y;
    // 己方 6 号空中机器人 x 位置坐标，单位：cm
    uint16_t ally_aerial_position_x;
    // 己方 6 号空中机器人 y 位置坐标，单位：cm
    uint16_t ally_aerial_position_y;
    // 己方哨兵机器人 x 位置坐标，单位：cm
    uint16_t ally_sentry_position_x;
    // 己方哨兵机器人 y 位置坐标，单位：cm
    uint16_t ally_sentry_position_y;
};

// 0x0306 自定义控制器与选手端交互数据
struct __attribute__((packed)) RefereeTxDataCustomClientData
{
    // bit 0-7：按键 1 通用键值
    uint16_t key_value_1 : 8;
    // bit 8-15：按键 2 通用键值；支持 2 键无冲
    uint16_t key_value_2 : 8;
    // bit 0-11：鼠标 X 轴绝对像素位置，屏幕左上角为 (0,0)
    uint16_t x_position : 12;
    // bit 12-15：鼠标左键状态，1 为按下，其他值为未按下
    uint16_t mouse_left : 4;
    // bit 0-11：鼠标 Y 轴绝对像素位置
    uint16_t y_position : 12;
    // bit 12-15：鼠标右键状态，1 为按下，其他值为未按下
    uint16_t mouse_right : 4;

    // 保留位
    uint16_t reserved;
};

// 0x0307 选手端小地图接收哨兵数据
struct __attribute__((packed)) RefereeTxDataMapData
{
    // 1：到目标点攻击 2：到目标点防守 3：移动到目标点
    uint8_t intention;
    // 路径起点 x 轴坐标，单位：dm
    uint16_t start_position_x;
    // 路径起点 y 轴坐标，单位：dm
    uint16_t start_position_y;
    // 路径点 x 轴增量数组，单位：dm
    int8_t delta_x[49];
    // 路径点 y 轴增量数组，单位：dm
    int8_t delta_y[49];
    // 发送方 id
    uint16_t sender_id;
};

// 0x0308 选手端小地图接收机器人数据
struct __attribute__((packed)) RefereeTxDataCustomInfo
{
    // 发送者的 id
    uint16_t sender_id;
    // 接收者的 id
    uint16_t receiver_id;
    // UTF-16 编码字符，支持中文，发送时注意大小端
    uint8_t user_data[30];
};

// 0x0309 自定义控制器接收机器人数据 图传链路
struct __attribute__((packed)) RefereeTxDataRobotCustomData
{
    uint8_t data[30];
};

// 0x0310 机器人发送给自定义客户端的数据
struct __attribute__((packed)) RefereeTxDataRobotClientData
{
    // 根据实际发送长度灵活处理，最大300
    uint8_t data[300];
};

// 0x0311 自定义客户端发送给机器人的自定义指令
struct __attribute__((packed)) RefereeRxDataClientRobotData
{
    // 自定义客户端发送给机器人的自定义指令，最大 30 字节
    uint8_t data[30];
};

/* Exported variables ---------------------------------------------------------*/

/* Exported function declarations ---------------------------------------------*/

/*************************** COPYRIGHT(C) SZTU-HJ ******************************/
