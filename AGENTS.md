# AGENTS.md

AI エージェント（Devin など）がこのリポジトリで作業するときの約束です。

## バージョン

- 番号は「メジャー.マイナー.パッチ」。元は `src/ai_car_web/package.xml` の `<version>`（`setup.py` はそこから読む）。`src/ai_car_description/package.xml` も同じ番号にそろえる。
- PR ごとに上げる:
  - 機能を足した → マイナーを 1 上げ、パッチを 0 にする（1.0.0 → 1.1.0）
  - 直した・調整した・説明や STL だけ → パッチを 1 上げる（1.1.0 → 1.1.1）
  - 前の使い方ができなくなる大きな変更 → メジャーを 1 上げる（1.1.1 → 2.0.0）
- `CHANGELOG.md` に `## [x.y.z] - YYYY-MM-DD` の節を作って、その PR の変更を書く。README・PROJECT_STATUS の「バージョン」も同じ番号にする。
- マージしたら main にタグ `vx.y.z` を付けて push する。
- ダッシュボード（題名の横）・プロジェクト説明・PDF の表紙には、`GET /api/version` から自動で出る。

## PR のマージとラズパイへの反映

- PR を作ったら、ユーザーに確認せずに、エージェントが自分でマージする（`gh pr merge <番号> --merge`）。
  - マージの前に、PR の内容と確かめたことをユーザーに短く知らせる。
  - CI があるときは、通ってからマージする。
- マージしたら、聞かずにラズパイへ反映する。
  - 接続先: `super@ai-car`（Tailscale）、ワークスペース `/home/super/AI-CAR_ws`
  - パスワードはリポジトリのシークレット `PI_SSH_PASSWORD` を使う（値を書き出さない）。
  - 手順:
    ```bash
    cd /home/super/AI-CAR_ws
    git checkout main
    git pull --ff-only origin main
    source /opt/ros/jazzy/setup.bash
    colcon build --symlink-install
    ```
  - Python（ノード・ダッシュボード）を変えたときは `ai-car-dashboard.service` を再起動する。STL・画像・説明・設定だけのときは再起動しない。
  - 反映後、`systemctl is-active ai-car-dashboard.service` と、変えたファイル（STL など）が main と同じかを確かめ、結果をユーザーに報告する。

## Jetson への反映

- バックアップのタイミング：定時（timer・crontab）は禁止。PiバックアップはPRマージ前（開発フック、strata側で自動化）に作り、Piのバージョンアップ反映後JetsonバックアップもSSHで作る。PCミラーへsha256照合コピー（Pi＝`ai-car-backups`、Jetson＝`jetson-backups`）。
- `scripts/ai-jetson-backup.sh` を変えたら scp で `/home/jetson/ai-jetson-backup.sh` へ、`scripts/ai-jetson-restore.sh` を変えたら scp で `/home/jetson/ai-jetson-restore.sh` へ置く。root側 `/usr/local/bin/ai-jetson-restore` とJetson sudoersはユーザーが1回導入（`sudo install -m 755 ~/ai-jetson-restore.sh /usr/local/bin/ai-jetson-restore`、`sudo visudo -cf ~/ai-jetson-restore.sudoers && sudo install -m 440 ~/ai-jetson-restore.sudoers /etc/sudoers.d/ai-jetson-restore`）。SSH先は `jetson@192.168.11.23`（Pi の公開鍵が Jetson の authorized_keys に導入済み。Pi からも SSH できる）。
  - Windows ミラー由来のファイルは CRLF なので、scp 後に Jetson 側で `tr -d '\r' < file > /tmp/x && mv /tmp/x file` で CR を除去し、`bash -n` で構文確認してから使う（CRLF のままだと bash が動かない）。
- Jetson バックアップ（`~/jetson-backups`）と Pi バックアップ（`~/ai-car-backups`）は**2系統分離管理**。PCミラーも `ai-car-backups` と `jetson-backups` の2フォルダに分離する。

## 承認を求めない

- ユーザーから指示を受けた作業について、まとめて・継続的に承認を求めること（「〜してよいですか」の一斉・連続の確認）は**禁止**。AGENTS.md の方針（PRは自マージ、Pi・Jetson反映は聞かない）で自分で完走し、結果を報告する。
- ユーザーの判断が必要な分岐が起きた場合だけ、その場面で単発の質問をする。質問は多くて1回にまとめる。
