# Chokudou_test


* while文内で，PA1がHighの間だけTIM2を回す．（PA1にはボタンを繋げた）  (ボタンを長押しで機構を動かす)

```c
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == GPIO_PIN_SET) {
      HAL_TIM_Base_Start_IT(&htim2);
    } else {
      HAL_TIM_Base_Stop_IT(&htim2);
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 999);
    }

```

* TIM2ではPID制御
* `PID_anti_windup.h`で速度制御時のPIDゲイン
* `PID_anti_windup_pos.h`で位置制御時のゲイン
* `PID_anti_windup.h`と`PID_anti_windup_pos.h`の`MAX_OUT`はTIM2の割り込みcallback関数内で制限してるduty比の最大値と同じ値にする
```c
  if (duty > 950) {
  duty = 950;
  } else if (duty < -950) {
  duty = -950;
  }
  if ((duty <= 0)) {
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (1000+(int)duty));
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
  //      HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
  } else {
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (1000-(int)duty));
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
  //      HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
  }
```
* mainの`private define` 内
```c
#define POS_FILTER_C 0.85f // 位置制御用のフィルター
#define SPEED_FILTER_C 0.7f // 速度制御用のフィルター

#define ADC_BOTTOM 800 // 機構が下にいるときのADCの値
#define ADC_TOP 3800 // 機構が上にいるときのADCの値
#define STROKE 0.3f // 直動のストローク(m)
#define TOGGLE_CONT_TH 0.25f // 位置制御に切り替える場所(m)
```
* `volatile float target_speed = 2.5;`で目標速度