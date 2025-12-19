了解です。
ここまでの作業を **「何を疑い → 何を実装し → 何が分かり → 次に何を打つ段階か」**という因果が分かる形でまとめます。

---

# ここまでの作業内容まとめ（技術的サマリ）

## 0. 出発点（問題設定）

* PyBullet + HunterSimulation 環境で **二脚ロボットが standing / crouch 姿勢で 1秒以上立てない**
* two_stage controller（warmup: position → hold: torque）を中心に検証
* ほぼすべての trial が **ABORT: tilt too large** で終了
* DONE（10秒完走）が一度も出ない

---

## 1. 制御・実装バグの切り分け（前半）

### 実施内容

* **position-only（warmup=999, blend=0）** で立てるかを確認
* q_err_rms（関節追従誤差）をログ・metrics・compare に追加
* gravity compensation / blend / kd_blend など制御系拡張を検証

### 分かったこと

* q_err_rms_mean ≈ 0.002 rad → **関節追従は正常**
* ゲインを上げても survival は 0.5〜0.9s で頭打ち
* よって **制御計算や関節対応ミスではない**

👉 問題は「制御以前の物理条件（姿勢・接触）」側と確定。

---

## 2. 姿勢仮説（直脚が不利）と crouch 導入

### 実施内容

* stance 概念を導入（standing / crouch_pos / crouch_neg）
* crouch_knee / crouch_ankle を YAML + sweep で駆動
* 局所探索（テンプレ近傍）まで実施

### 分かったこと

* standing: survival ≈ 0.54s
* crouch: survival ≈ 0.88–0.89s（明確に改善）
* ただし **1秒の壁は突破できない**
* 深くしても浅くしても改善は限定的

👉 **「直脚姿勢が悪い」仮説は正しいが、根本原因ではない**。

---

## 3. safety 閾値の観測緩和（切り捨て判定か？）

### 実施内容

* safety.max_roll / max_pitch を sweep で緩和
* compare に s_roll / s_pitch 表示追加

### 分かったこと

* 閾値を 0.7 → 1.0 rad にすると survival は 0.89 → 0.96s まで伸びる
* しかし **ロール角は新しい閾値まで単調増大して abort**
* 回復（収束）挙動は見られない

👉 **安全判定が厳しすぎるだけではない**。
👉 ロールが物理的に発散している。

---

## 4. 接触・滑り・足幅の観測を追加（本丸）

### 実施内容

* samples に以下を追加：

  * fz_left/right, fxy_left/right
  * foot_lx/ly/lz, foot_rx/ry/rz, foot_dy
  * slip_left/right（ステップ間）
* metrics / compare に表示
* link-contact histogram（どのリンクが接触しているか）を追加

### 分かったこと（決定的）

* **ct_any ≈ 0.0167**

  * 制御更新の約 1/60 でしか接触が取れていない
* fz_left/right ≈ 0（ほぼ常時）
* foot_dy_min ≈ 3e-5（左右足の間隔が潰れてほぼ 0）
* 接触が取れた瞬間は foot_link が top_contact に出るが、極めて稀

👉 **ロボットは「地面に安定して乗っていない」**
👉 滑っている以前に、**接触そのものが成立していない時間が大半**

---

## 5. BASE_HEIGHT sweep（接触成立条件の探索）

### 実施内容

* BASE_HEIGHT を YAML / sweep から駆動
* 粗 sweep → 近傍 sweep を実施

### 分かったこと

* 0.679m 付近だけが明確に良い

  * survival ≈ 0.89s
  * ct_any が非ゼロになる唯一の点
* ±0.01m で急激に悪化
* **接触成立条件が極端にシビア**

👉 高さは「正しい」が、**それだけでは足りない**。

---

## 6. 現時点での最終結論（状態把握）

### 技術的に確定したこと

1. **制御（PD / MPC / torque / gravity）に致命的な問題はない**
2. **関節追従は非常に良好**
3. **ロール発散の主因は「足幅が潰れること」**

   * foot_dy → 0
   * 結果として片脚支持・無接触状態が頻発
4. **接触が成立しない時間が長すぎる**

   * ct_any ≪ 1
5. crouch は有効だが、**初期足配置（hip roll）が狭すぎる**

---

## 7. 次に打つべき一手（現在進行中）

### 方針

* **足幅（hip roll）を明示的に広げる**
* stance テンプレに上乗せする形で `hip_roll_delta` を導入

### 実装済み

* `controller.hip_roll_delta` を YAML / sweep から駆動
* stance_q_ref で

  * leg_l1 -= delta
  * leg_r1 += delta
* compare に hip_roll_delta 列を追加

### 次の検証コマンド（要点）

```bash
--base-height 0.679 \
--hip-roll-delta 0.00,0.03,0.06,0.09,0.12
```

### 成功判定指標

* foot_dy_min が 0 から明確に増える
* ct_any が 0.02 → 0.2 以上に増える
* survival が 1秒を突破

---

## 全体像（1行まとめ）

> **「立てない原因は制御ではなく、初期姿勢が狭すぎて足接触が成立しないこと。
> BASE_HEIGHT と crouch で“高さ”は合ったが、“足幅（hip roll）”が最後の本丸」**

この状態まで切り分けられているのは、かなり理想的です。
次は **hip_roll_delta sweep** で、初めて「物理的に立てる条件」に到達する可能性が高い段階です。


To continue this session, run codex resume 019b313c-8b9d-7942-b7b5-3aa1a19de031
