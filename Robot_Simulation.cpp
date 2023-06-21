#include <stdio.h>
#include <math.h>
#include <iostream>

//人間初期位置
double x_start = 0.0;
double z_start = 4.0;

double one_frame = 0.1;
double human_speed = 1.0; //(m/s)
double before[5] = { 1.0, 1.0, 1.0, 1.0, 1.0 };

//移動の変数
int human_mode = 1;
int arm_mode = 1;
int started = 0;
double return_up = 1.25; //アーム折り返し地点
double return_down = 0.75; //アーム折り返し地点
double stop_count_head;
double stop_count_Lighthandhead;
double stop_count_Lefthand;
double straight = 0;
double app_or_leave = 1; //1は接近、2は離れる
//変更
//アームの動き
int arm_movetype = 1; //1は上から、２は下から、３は中央から上、４は中央から下
//人間の動き
int human_movetype = 3; //1はコンテナ回収、2は果房作業、3なら隣のレーン

//出力用の変数
int Data_Start = 0;
//ファイル名の変数、今回はcsv形式で出力するので最後に拡張子.csvをつける
char fname[] = "All result6.csv";

//危険度関数に関する変数
int Danger_type = 0; //1のとき段階的に上がる
double Base_α = 100; //aの最大値
double Base_β = 100; //βの最大値

//作業効率
double Short_efficiency = 1; //短期間の作業効率
double Short_danger = 0; //短時間の危険度平均
double StartTime = 0.7; //観測開始時間
double StopTime = 5.2; //観測終了時間
double leftTotal = 0; //作業効率の分子
double rightTotal = 0; //作業効率の分母
double Total_danger = 0;
double Stop_time = 0; //アームが停止してる時間

//座標を表す構造体
typedef struct {
    double x;
    double y;
    double z;
    double speed;
    double a; //直線
    double b;
    double c;
    double d; //垂直直線
    double e;
    double f;
    double p; //交点座標
    double q;
} Position;

//危険度関数を表す構造体
typedef struct {
    double danger;
    double K1;
    double K2;
    double α;
    double β;
    double start;
    double limit;
    double grade;
} Danger;

//アーム初期設定
bool armShoki(int a, double* z, int* type) {
    if (a == 1) {
        *z = return_up;
        *type = 2;
    }
    if (a == 2) {
        *z = return_down;
        *type = 1;
    }
    if (a == 3) {
        *z = (return_down + return_up) / 2;
        *type = 1;
    }
    if (a == 4) {
        *z = (return_down + return_up) / 2;
        *type = 2;
    }

    return true;
}

//人の初期位置を設定する関数
bool humanShoki(int type, double* x, double* count, double* straight) {
    if (type == 1) {
        *x = 0.0;
        *count = 20;
    }
    if (type == 2) {
        *x = 0.3;
        *count = 50;
    }
    if (type == 3) {
        *x = 0.85;
        *straight = 1;
    }

    return true;
}

//X-Z平面上の距離を計算する関数
double distance(double armX, double armY, double armZ, double HumanX, double HumanY, double HumanZ) {

    double distanceX = armX - HumanX;
    double distanceY = armY - HumanY;
    double distanceZ = armZ - HumanZ;

    double distanceXYZ = sqrt(pow(distanceX, 2) + pow(distanceY, 2) + pow(distanceZ, 2));

    return distanceXYZ;
}

//危険度を計算する関数
double calc_Danger(double K1, double K2, double L1, double L2, double average_v, double app_or_leave) {

    double DangerPower;
    if (app_or_leave == 1) {
        L1 -= average_v * one_frame;
        L2 -= average_v * one_frame;
        DangerPower = K1 / pow(L1 * 1000, 2) + K2 / pow(L2 * 1000, 2);
    }

    if (app_or_leave == 2) {
        L1 += average_v * one_frame;
        L2 += average_v * one_frame;
        DangerPower = K1 / pow(L1 * 1000, 2) + K2 / pow(L2 * 1000, 2);
    }
    return DangerPower;
}

//αを計算する関数
double Calc_α(double α_distance, bool Inside) {
    double value;
    if (α_distance < 1.0 && Inside == 0) {
        value = (1 - α_distance) * Base_α;
    }
    //領域α内部にある
    if (α_distance < 1.0 && Inside == 1) {
        value = Base_α;
    }
    if (1.0 <= α_distance) {
        value = 0;
    }
    return value;
}

//XZの距離を計算する関数
double XZ_distace(double armJiku_x, double armJiku_z, double human_x, double human_z) {

    double distanceX = armJiku_x - human_x;
    double distanceZ = armJiku_z - human_z;

    double distanceXZ = sqrt(pow(distanceX, 2) + pow(distanceZ, 2));

    return distanceXZ;
}

//βを計算する関数
double Calc_β(double β_distance) {
    double value;
    if (1.504 < β_distance) {
        value = 0;
    }
    if (0.504 <= β_distance && β_distance <= 1.504) {
        value = ((1.504 - β_distance) / (1.504 - 0.504)) * Base_β;
    }
    if (β_distance < 0.504) {
        value = Base_β;
    }
    return value;
}

//危険度によってアームの速度を調整する関数
//危険度25から減速開始
double armSpeedcontrol(double danger, int type) {

    double speed = 0.25;

    if (100 <= danger) {
        return speed = 0;
    }
    if (25 <= danger && danger < 100) {
        if (type == 0) {
            return speed = 0.25 * ((100 - danger) / (100 - 25));
        }
        if (type == 1) {
            if (30 < danger && danger <= 40) {
                return speed = 0.25 * 0.8;
            }
            if (40 < danger && danger <= 50) {
                return speed = 0.25 * 0.6;
            }
            if (50 < danger && danger <= 60) {
                return speed = 0.25 * 0.4;
            }
            if (60 < danger && danger <= 70) {
                return speed = 0.25 * 0.2;
            }
        }
    }
    if (danger < 25) {
        return speed = 0.25;
    }

    return speed;
}

//人座標(x,z)が三角形OAB内部にあるかを判定する関数
bool isInsideTriangle(Position O, Position A, Position B, double x, double z) {
    //三角形OABの面積を求める
    double S = std::abs((A.x - O.x) * (B.z - O.z) - (A.z - O.z) * (B.x - O.x)) / 2;

    //三角形OAP,OBP,ABCの面積を求める
    double S_OAP = std::abs((A.x - O.x) * (z - O.z) - (A.z - O.z) * (x - O.x)) / 2;
    double S_OBP = std::abs((B.x - O.x) * (z - O.z) - (B.z - O.z) * (x - O.x)) / 2;
    double S_ABP = std::abs((A.x - x) * (B.z - z) - (A.z - z) * (B.x - x)) / 2;

    //三角形OAP、OBP、ABPの面積が三角形OABの面積と等しければ、点P(x,z)は三角形OABの内部にある。
    if (S == (S_OAP + S_OBP + S_ABP)) {
        return true;
    }
    return false;
}

double get_angle(Position p1, Position p2) {
    double dx = p2.x - p1.x;
    double dy = p2.z - p1.z;
    return atan2(dy, dx);
}

double get_point_line_angle(Position p, Position l1, Position l2) {
    double line_angle = get_angle(l1, l2);
    double point_angle = get_angle(l1, p);
    return point_angle - line_angle;
}

//人座標(x,z)から三角形OABの辺上、内部に最も近い点との距離を求める関数
double α_area(Position O, Position A, Position B, Position human) {
    //三角形OABの辺OA、OB、ABに最も近い距離を求める
    double x = human.x;
    double z = human.z;
    //三角形の各点との距離
    double Ap_distance = sqrt(pow((x - A.x), 2) + pow((z - A.z), 2));
    double Bp_distance = sqrt(pow((x - B.x), 2) + pow((z - B.z), 2));
    double Op_distance = sqrt(pow((x - O.x), 2) + pow((z - O.z), 2));

    double α_distance = XZ_distace(A.x, A.z, human.x, human.z);

    if (Ap_distance <= 1.0 || Bp_distance <= 1.0 || Op_distance <= 1.0) {

        double min_value = Ap_distance;
        if (Bp_distance < min_value) min_value = Bp_distance;
        if (Op_distance < min_value) min_value = Op_distance;

        α_distance = min_value;
        //A_value = (1.00 - min_value) * Base_α;
    }

    //線分から平行のチェック
    //A B
    if ((B.x <= x && x <= B.x + 1.0) && (B.z <= z && z <= A.z)) {
        double dP = x - B.x;
        α_distance = dP;
        //A_value = (1 - dP) * Base_α;
    }

    //A O
    double angle1 = get_point_line_angle(human, O, A) * 180.0 / M_PI;
    double angle2 = get_point_line_angle(human, A, O) * 180.0 / M_PI;
    if ((0 <= angle1 && angle1 <= 90) && (270 <= angle2 && angle2 <= 360)) {
        //線文を通る著k線の方程式の係数
        double katamuki = (O.z - A.z) / (O.x - A.x);
        double seppen = A.z - katamuki * A.x;

        //点Pと直線の距離
        double d = abs(katamuki * human.x + seppen - human.z) / sqrt(katamuki * katamuki + 1);
        if (d <= 1.0) {
            α_distance = d;
            //A_value = (1 - d) * Base_α;
        }
    }
    //B 0
    double angle3 = get_point_line_angle(human, O, B) * 180.0 / M_PI;
    double angle4 = get_point_line_angle(human, B, O) * 180.0 / M_PI;
    if ((-90 <= angle3 && angle3 <= 0) && (-360 <= angle4 && angle4 <= -270)) {
        //線文を通る著k線の方程式の係数
        double katamuki = (O.z - B.z) / (O.x - B.x);
        double seppen = A.z - katamuki * B.x;

        //点Pと直線の距離
        double d = abs(katamuki * human.x + seppen - human.z) / sqrt(katamuki * katamuki + 1);
        if (d <= 1.0) {
            α_distance = d;
            //A_value = (1 - d) * Base_α;
        }
    }

    return α_distance;
}

//短期的な作業効率を計算する関数
bool Calc_Short_efficiency(double time, double armspeed, double* leftTotal, double* rightTotal, double* Short_efficiency, double* Short_danger, double* Total_danger, double danger) {
    if (StartTime <= time && StopTime >= time) {
        *leftTotal += armspeed;
        *rightTotal += 0.25;
        *Short_efficiency = *leftTotal / *rightTotal;

        *Total_danger += danger;
        *Short_danger = *Total_danger / ((StopTime - StartTime) * 10);

        return true;
    }
    return true;
}

//データをcsvに出力する関数
void Record_data(double Total_time, double danger, double Work_efficiency, double armspeed, double Human_Robot_Distance, double armz, double humanx, double humanz, double a_distance, double b_distance, double a, double b, double stopcount, double Short_efficiency, double app_or_leave, double Short_danger, double Stop_time, double Od) {
    if (Data_Start == 0) {
        FILE* fs;
        fs = fopen(fname, "a");
        fprintf(fs, "Total_time, human.z, danger, armspeed, arm.z, Work_efficiency, Human_Robot_Distance, humanx, a_distance, b_distance, a, b, stopcount, Short_efficiency, app_or_leave, Short_danger, Stop_time, O.d\n");
        fclose(fs);

        Data_Start = 1;
    }

    FILE* fs;
    fs = fopen(fname, "a");
    fprintf(fs, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf\n", Total_time, humanz - 1, danger, armspeed, armz - 1, Work_efficiency, Human_Robot_Distance, humanx, a_distance, b_distance, a, b, stopcount, Short_efficiency, app_or_leave, Short_danger, Stop_time, Od);
    fclose(fs);
}

//アームZ座標移動 0.75~1.25mを往復
double move_arm(double z, double speed) {

    double armZ = z;
    double move_distance = speed * one_frame;

    if (arm_mode == 1) {
        armZ += move_distance;
    }
    if (arm_mode == 2) {
        armZ -= move_distance;
    }

    if (return_up < armZ) {
        arm_mode = 2;
    }
    if (armZ < return_down) {
        arm_mode = 1;
    }

    return armZ;
}

//人を移動する関数
double move_human(double z, double speed, double* stop_count, double stop_count_value, double* app_or_leave, double straight) {

    double i = 1;
    double humanZ = z;
    double value = stop_count_value;
    double move_distance = speed * one_frame;
    double return_point = 1.5;

    //真っ直ぐ進む
    if (straight == 1) {
        humanZ -= move_distance;
        return humanZ;
    }

    if (human_mode == 1) {
        humanZ -= move_distance;
    }
    if (human_mode == 2) {
        humanZ += move_distance;
    }

    if (humanZ < return_point) {
        human_mode = 3;
        *app_or_leave = 2;
    }

    if (human_mode == 3) {
        *stop_count = value - 1;

        if (value <= 1.0) {
            human_mode = 2;
        }

    }

    return humanZ;
}

int main(void) {

    //危険度
    Danger Danger;
    Danger.danger = 0;
    Danger.K1 = 50 * pow(10, 6);
    Danger.K2 = 50 * pow(10, 6);
    Danger.α = 0;
    Danger.β = 0;
    Danger.start = 25;
    Danger.limit = 100;
    Danger.grade = 0;

    //人初期位置
    Position human;
    human.x = 0 + x_start;
    human.y = 1.6;
    human.z = z_start;
    human.speed = human_speed;

    //手初期位置
    Position LeftHand;
    LeftHand.x = 0.25 + x_start;
    LeftHand.y = 1.0;
    LeftHand.z = z_start;

    Position RightHand;
    RightHand.x = -0.25 + x_start;
    RightHand.y = 1.0;
    RightHand.z = z_start;

    //エンドエフェクタ初期位置
    Position arm;
    arm.x = 0.346;
    arm.y = 1.47;
    arm.z = 1.0;
    arm.speed = 0.25;

    //エンドエフェクタ軸
    Position armJiku;
    armJiku.x = 0.0;
    armJiku.y = 1.0;
    armJiku.z = 1.0;

    //領域α
    //点O（アーム軸）
    Position O;
    O.x = armJiku.x;
    O.z = armJiku.z;
    O.d = 0;

    //点A
    Position A;
    A.x = arm.x;
    A.z = return_up;

    //点B
    Position B;
    B.x = arm.x;
    B.z = return_down;

    //出力する値
    double Total_time = 0;
    double Total_arm = 0;
    double Work_efficiency = 1;
    double Control = 0;
    double Total_arm_No_Control = 0;

    while (1) {

        //最初の設定
        if (started == 0) {
            armShoki(arm_movetype, &arm.z, &arm_mode);
            double stop_count;
            humanShoki(human_movetype, &x_start, &stop_count, &straight);
            stop_count_head = stop_count;
            stop_count_Lighthandhead = stop_count;
            stop_count_Lefthand = stop_count;
            human.x = 0 + x_start;
            LeftHand.x = 0.25 + x_start;
            RightHand.x = -0.25 + x_start;
            started = 1;
        }

        //5フレーム間の平均速度を求める
        for (int k = 4; 1 <= k; k--) {
            before[k - 1] = before[k];
        }
        before[4] = human.speed;

        double v = 0;
        for (int i = 0; i <= 4; i++) {
            v += before[i];
        }
        double average_v = v / 5;

        //頭部とエンドエフェクタ間のX-Z平面上の距離（ｍ）
        double L1 = distance(arm.x, arm.y, arm.z, human.x, human.y, human.z);

        //左手とエンドエフェクタ間のX-Z平面上の距離（ｍ）
        double L2_Left = distance(arm.x, arm.y, arm.z, LeftHand.x, LeftHand.y, LeftHand.z);
        double L2_Right = distance(arm.x, arm.y, arm.z, RightHand.x, RightHand.y, RightHand.z);
        double L2 = (std::min)(L2_Left, L2_Right);

        //危険度検出
        Danger.danger = calc_Danger(Danger.K1, Danger.K2, L1, L2, average_v, app_or_leave);
        std::cout << Danger.danger << std::endl;

        //領域α
        double α_distance = α_area(O, A, B, human);
        bool Inside = isInsideTriangle(O, A, B, human.x, human.z);
        Danger.α = Calc_α(α_distance, Inside);
        //double α_distance = 0;

        //領域αを足す
        Danger.danger += Danger.α;

        //頭部とアーム軸との距離
        double β_distance = XZ_distace(armJiku.x, armJiku.z, human.x, human.z);
        //βが1m以内なら危険度に+β
        Danger.β = Calc_β(β_distance);

        //領域βを足す
        Danger.danger += Danger.β;

        //危険度→エンドエフェクタ移動距離（m/s)
        double armspeed = armSpeedcontrol(Danger.danger, Danger_type);
        arm.speed = armspeed;
        if (arm.speed == 0) {
            Stop_time += 0.1;
        }

        //各値をCSVに出力
        Record_data(Total_time, Danger.danger, Work_efficiency, arm.speed, β_distance, arm.z, human.x, human.z, α_distance, β_distance, Danger.α, Danger.β, stop_count_head, Short_efficiency, app_or_leave, Short_danger, Stop_time, O.b);

        //シミュレーションのトータル時間
        Total_time += one_frame;

        //制御したアームが動いた距離
        Total_arm += arm.speed * one_frame;

        //制御なしのアームが動いた距離
        Total_arm_No_Control += 0.25 * one_frame;

        //作業効率
        Work_efficiency = Total_arm / Total_arm_No_Control;

        //短期的な作業効率
        Calc_Short_efficiency(Total_time, armspeed, &leftTotal, &rightTotal, &Short_efficiency, &Short_danger, &Total_danger, Danger.danger);

        //アームを移動
        double armZ = move_arm(arm.z, arm.speed);
        arm.z = armZ;

        //人をｚ方向へ移動
        double humanZ = move_human(human.z, human.speed, &stop_count_head, stop_count_head, &app_or_leave, straight);
        human.z = humanZ;

        double LeftHandZ = move_human(LeftHand.z, human.speed, &stop_count_Lighthandhead, stop_count_Lighthandhead, &app_or_leave, straight);
        double RightHandZ = move_human(RightHand.z, human.speed, &stop_count_Lefthand, stop_count_Lefthand, &app_or_leave, straight);
        LeftHand.z = LeftHandZ;
        RightHand.z = RightHandZ;

        //人が元の場所に戻ると終了
        if (z_start < human.z && human_movetype == 1) {
            break;
        }
        if (z_start < human.z && human_movetype == 2) {
            break;
        }
        //人が-2mのところまで来ると終了
        if (human.z < -2 && human_movetype == 3) {
            break;
        }
    }

    return 0;
}
