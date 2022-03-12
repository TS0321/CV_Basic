メンバ関数の後ろにconstをつけると、その関数内ではメンバ変数を変更できなくなる
```c++
class CameraParam
{
public:
    cv::Mat getCvCamParam() const;
}
```

