# rogicameraflex_ros2

ROS2用の柔軟なカメラ画像処理パッケージです。1つのカメラからキャプチャした画像に対して、複数の異なる画像処理パイプラインを並列実行し、それぞれ異なるトピックに配信することができます。

## 特徴

- **柔軟なパイプライン設定**: YAMLファイルで複数の画像処理パイプラインを設定可能
- **複数トピック対応**: 1つのカメラから複数の異なる処理済み画像を同時配信
- **豊富な画像処理**: クロップ、リサイズ、色空間変換などの基本的な画像処理をサポート
- **設定ベース**: コードを変更せずに設定ファイルのみで処理内容を変更可能

## サポートしている画像処理

- **crop**: 画像の指定領域を切り出し
- **resize**: 画像のサイズ変更
- **color_convert**: 色空間変換（HSV、グレースケールなど）

## 依存関係

- ROS2 (Humble/Iron/Rolling)
- OpenCV
- yaml-cpp

### ROS2パッケージ依存

- `rclcpp`
- `sensor_msgs`
- `cv_bridge`
- `image_transport`

## インストール

```bash
# ワークスペースに移動
cd ~/ros2_ws/src

# パッケージをクローン（またはコピー）
# git clone <repository_url> rogicameraflex_ros2

# ワークスペースのルートに戻る
cd ~/ros2_ws

# ビルド
colcon build --packages-select rogicameraflex_ros2

# 環境変数を読み込み
source install/setup.bash
```

## 設定

`config/config.yaml`ファイルで動作を設定します。

### カメラ設定

```yaml
camera:
  device: 0              # カメラデバイス番号
  frame_id: "camera_frame"
  width: 640             # キャプチャ解像度
  height: 480
  fps: 30               # フレームレート
```

### パイプライン設定例

```yaml
pipelines:
  # クロップパイプライン
  - name: "crop_pipeline"
    topic: "/camera/crop/image_raw"
    operations:
      - type: crop
        x_offset: 100      # 左からの開始位置(px)
        y_offset: 50       # 上からの開始位置(px)
        width: 400         # 切り出し幅(px)
        height: 300        # 切り出し高さ(px)

  # HSV変換パイプライン
  - name: "hsv_pipeline"
    topic: "/camera/hsv/image_raw"
    operations:
      - type: color_convert
        encoding: "HSV"

  # 複数操作の組み合わせ例
  - name: "complex_pipeline"
    topic: "/camera/processed/image_raw"
    operations:
      - type: crop
        x_offset: 50
        y_offset: 50
        width: 500
        height: 400
      - type: resize
        width: 320
        height: 240
      - type: color_convert
        encoding: "GRAY"
```

## 使用方法

### 1. 設定ファイルの編集

`config/config.yaml`を編集して、カメラ設定と処理パイプラインを定義します。

### 2. ノードの起動

```bash
# デフォルト設定で起動
ros2 run rogicameraflex_ros2 camera_flex_node

# カスタム設定ファイルを指定
ros2 run rogicameraflex_ros2 camera_flex_node --ros-args -p config_file:=/path/to/your/config.yaml
```

### 3. 画像の確認

```bash
# 利用可能なトピックを確認
ros2 topic list | grep image

# 画像を表示（例：クロップされた画像）
ros2 run rqt_image_view rqt_image_view /camera/crop/image_raw

# または rviz2 で確認
rviz2
```

## 画像処理操作の詳細

### crop（クロップ）
指定した矩形領域を切り出します。
```yaml
- type: crop
  x_offset: 100    # X軸オフセット
  y_offset: 50     # Y軸オフセット  
  width: 400       # 切り出し幅
  height: 300      # 切り出し高さ
```

### resize（リサイズ）
画像サイズを変更します。
```yaml
- type: resize
  width: 320       # 新しい幅
  height: 240      # 新しい高さ
```

### color_convert（色空間変換）
色空間を変換します。
```yaml
- type: color_convert
  encoding: "HSV"  # "HSV" または "GRAY"
```

## トラブルシューティング

### カメラが開けない場合
- デバイス番号を確認してください（通常は0、1、2...）
- カメラが他のアプリケーションで使用されていないか確認してください
- USBカメラの場合、接続を確認してください

### 設定ファイルが読み込めない場合
- ファイルパスが正しいか確認してください
- YAMLの構文エラーがないか確認してください
- ファイルの読み取り権限があるか確認してください

### 画像が表示されない場合
- トピック名が正しく設定されているか確認してください
- `ros2 topic list`でトピックが存在するか確認してください
- `ros2 topic echo /camera/crop/image_raw`でデータが流れているか確認してください

## ライセンス

TODO: ライセンスを記載してください

## 貢献

バグ報告や機能追加の提案は、GitHubのIssueまたはPull Requestでお願いします。

## 作者

- メンテナー: root (ymrs1122@gmail.com)