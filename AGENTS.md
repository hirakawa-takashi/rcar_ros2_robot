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
