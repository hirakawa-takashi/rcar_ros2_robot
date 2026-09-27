# AGENTS.md

AI エージェント（Devin など）がこのリポジトリで作業するときの約束です。

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
