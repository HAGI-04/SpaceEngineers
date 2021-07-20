//===================================================================//
//                                                                   //
//                  ロボットの脚を制御するプログラム                    //
//                                                                   //
//===================================================================//

//課題
// コクピットからの入力でtimelineが変化するようにする
// X, Y, H の設定値を考える
// 左右移動、方向転換ができるようにする
// 各脚の位置を初期化するメソッドを作る
// ジャンプ機能をつける（一瞬だけHを大幅に下げればいいと思う）
// しゃがみ機能をつける（ボタンを押している間Hを上げる）（コクピットに人が乗っていないときはしゃがむ）


const double A = 2d, B = 4d, C = 9.5;   // 各ローター間の長さ
const float Kp = 15f;    // P制御の係数
const double legCycleSpeed = 1d;    //足の移動速度の倍率（デフォは1）
const string cockpitName = "ROBOT-Cockpit";
const string LegName = "Leg";

Leg[] legs = new Leg[4];    // 脚の構造体

//=======================================================================================================================================

struct Leg
{
    public double X;    // X座標 コクピット基準で右側が正
    public double Y;    // Y座標 コクピット基準で正面側が正
    private double H;   // 高さ（コクピット基準なので常に負の値）
    private double R;   // 真上から見た足の長さ
    public string keyword;  // ローター検索用キーワード
    public double cycleTime;        // 脚のサイクルの位置
    public double cycleTimeOffset;  // 各脚でサイクルをずらすためのオフセット量
    public bool isFrontSide;    // 脚がついているのは正面側か
    public bool isRightSide;    // 脚がついているのは右側か

    public IMyMotorStator[] rotor;  // 関節のローター
    public double[] rotorAngle;    // 各ローターの角度の設定値

    //-----------------------------------------------------------------------------

    // ローター3つ, X, Y をセットした後に呼ぶことで脚を所定の位置に動かす
    public void moveAllRotorTo_X_Y_Z(double timeline)
    {
        calc_X_Y(timeline);
        calc_R_H();
        calcRotorAngle();
        syncRotorAngle(rotor[0], rotorAngle[0]);
        syncRotorAngle(rotor[1], -1 * rotorAngle[1]);
        syncRotorAngle(rotor[2], -1 * ((-1 * rotorAngle[1]) + rotorAngle[2]));
    }

    //-----------------------------------------------------------------------------

    // 受け取ったtimelineと各脚の取り付け位置によって脚の先端の位置を決める（仮）
    private void calc_X_Y(double timeline)
    {
        // cycleTimeの決定
        cycleTime = (timeline + cycleTimeOffset) % 100;
        if(!isFrontSide)
            cycleTime = (125 - cycleTime) % 100;

        // X の設定（仮）
        X = 7d;

        if(!isRightSide)
            X *= -1d;

        // Y の設定（仮）
        if(cycleTime < 25d)
            Y = (8d / 25d) * cycleTime + 2d;
        else
            Y = (-8d / 75d) * cycleTime + (38d / 3d);

        if(!isFrontSide)
            Y *= -1d;
    }

    //-----------------------------------------------------------------------------

    // X, Y から R, H を計算してセットする
    private void calc_R_H()
    {
        R = Math.Sqrt(Math.Pow(X, 2) + Math.Pow(Y, 2));

        // cycleTime:0~100, 0~75:接地, 75~100:原点回帰（仮）
        if(cycleTime < 25)
            H = 4 * Math.Sin(cycleTime * Math.PI / 25d) - 4;
        else
            H = -4d;
    }

    //-----------------------------------------------------------------------------

    // X, Y, H, R から各ローターの角度を計算してセットする
    private void calcRotorAngle()
    {
        rotorAngle[0] = Math.Atan(Y / X);
        calc_theta_b_and_theta_c(R, H, out rotorAngle[1], out rotorAngle[2]);

        if(!(isFrontSide ^ isRightSide))
        {
            rotorAngle[1] *= -1d;
            rotorAngle[2] *= -1d;
        }
    }

    //-----------------------------------------------------------------------------

    //  R, H を渡し、theta_b, theta_c を計算する
    //  引数の角度の単位はラジアン
    private void calc_theta_b_and_theta_c(double R, double H, out double theta_b, out double theta_c)
    {
        double k_a = A, b_b = B, b_c = C;
        double theta_a = 0, v_e, theta_e;
        double k_d = Math.Sqrt(Math.Pow(R, 2) + Math.Pow(H, 2)), theta_d = Math.Atan(H / R);

        theta_e = Math.Atan( (k_d * Math.Sin(theta_d) - k_a * Math.Sin(theta_a)) / (k_d * Math.Cos(theta_d) - k_a * Math.Cos(theta_a)) );
        v_e = k_d * Math.Cos(theta_d - theta_e) - k_a * Math.Cos(theta_a - theta_e);

        double cos_theta_b_minus_theta_e = (Math.Pow(b_c, 2) - Math.Pow(b_b, 2) - Math.Pow(v_e, 2)) / ((-2) * b_b * v_e);
        double sin_theta_b_minus_theta_e = Math.Sqrt(1 - Math.Pow(cos_theta_b_minus_theta_e, 2));
        double tan_theta_b_minus_theta_e = sin_theta_b_minus_theta_e / cos_theta_b_minus_theta_e;

        if(tan_theta_b_minus_theta_e < 0)
            theta_b = Math.Atan(tan_theta_b_minus_theta_e) + Math.PI + theta_e;
        else
            theta_b = Math.Atan(tan_theta_b_minus_theta_e) + theta_e;
        
        theta_c = Math.Atan( (b_b * Math.Sin(theta_b) - v_e * Math.Sin(theta_e)) / (b_b * Math.Cos(theta_b) - v_e * Math.Cos(theta_e)) );
    }

    //-----------------------------------------------------------------------------

    // 目標角度を渡し、ローターをP制御で所定位置に動かす
    // 引数の角度の単位はラジアン
    private void syncRotorAngle(IMyMotorStator rotor, double destinationAngle)
    {
        float angleDeviation = (float)destinationAngle - rotor.Angle;
        if(angleDeviation > Math.PI)
            angleDeviation -= 2 * (float)Math.PI;
        if(angleDeviation < -1 * Math.PI)
            angleDeviation += 2 * (float)Math.PI;

        if(rotor.GetValueBool("OnOff"))
            rotor.TargetVelocityRad = Kp * angleDeviation;
    }
}

//=======================================================================================================================================

public Program()
{
    Storage = "0";    // timeline

    // コクピットの取得
    IMyShipControllerIMyShipController cockpit = GridTerminalSystem.GetBlockWithName (cockpitName) as IMyShipController;

    // 各脚の初期設定
    for(int i = 0; i < legs.Length; i++)
    {
        legs[i].rotor = new IMyMotorStator[3];
        legs[i].rotorAngle = new double[3];
        legs[i].cycleTimeOffset = 25d * i;
        legs[i].keyword = LegName + (i+1).ToString();
        for(int j = 0; j < 3; j++)
            legs[i].rotor[j] = GridTerminalSystem.GetBlockWithName(legs[i].keyword + "-" + (j+1).ToString()) as IMyMotorStator;
    }

    // 各脚の取り付け位置を設定する
    legs[0].isFrontSide = false;
    legs[0].isRightSide = false;

    legs[1].isFrontSide = true;
    legs[1].isRightSide = true;

    legs[2].isFrontSide = true;
    legs[2].isRightSide = false;
    
    legs[3].isFrontSide = false;
    legs[3].isRightSide = true;
}

//=======================================================================================================================================

public void Main(string argument, UpdateType updateSource)
{
    // 実行間隔の設定
    Runtime.UpdateFrequency = UpdateFrequency.Update1;

    // timelineの取得
    double timeline = double.Parse(Storage);

    // 各脚を目標地点へ動かす
    for(int i = 0; i < legs.Length; i++)
        legs[i].moveAllRotorTo_X_Y_Z(timeline);

    // timelineの更新処理（仮）
    timeline += -1d * cockpit.MoveIndicator.Z * legCycleSpeed;
    //timeline += 0.5;
    if(timeline < 0)    timeline += 100;
    if(100 < timeline)  timeline -= 100;
    
    // timelineの保存
    Storage = timeline.ToString();
}

//=======================================================================================================================================