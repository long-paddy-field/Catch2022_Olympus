# パッケージ　Catch2022_joy_commander 概要

## 構成ノード
- joy_commander
左スティックと左十字の入力を関節の回転角に変換

- joy_controller
ジョイコンの入力をトピックとしてpublish

## joy_controller ボタン配置

| ボタン番号 | ボタン | 対応動作 | topic | 型 |
|----------| ----- | ------- | ----- | -- |
| 0 | X | 青コーナー設定 | field_color | std_msgs String |
| 1 | A | 掴む | grab_cmd | std_msgs Empty |
| 2 | B | 赤コーナー設定 | field_color | std_msgs String |
| 3 | Y | 離す | release_cmd | std_msgs Empty |
| 4 | LB | サーボCCW | servo_cmd | Int8 |
| 5 | RB | サーボCW | servo_cmd | Int8 |
| 6 | LT | ステッパ上昇 | step_cmd | Int8 |
| 7 | RT | ステッパ下降 | step_cmd | Int8 |
| 8 | BACK | 緊急停止(ソフト) | emergence_flag | std_msgs Bool |
| 9 | START | 初期化処理の終了 | start_flag | std_msgs Empty |
| 10 | 左押し込み | 未定 | 未定 | 未定 |
| 11 | 右押し込み | 手動自動切り替え | is_handy | std_msgs Bool |

