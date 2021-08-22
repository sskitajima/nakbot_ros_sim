# ROSチュートリアル第2回

[第1回のリポジトリ](https://github.com/sskitajima/ros_tutorial)

## はじめに

#### 目的

ROSとGazeboを用いたシミュレーションについて、その基礎を**実際に手を動かすことで確実に**身につけること。具体的には以下の習得を目的とする。

- Gazeboの操作に慣れる
- worldの基本的な作成方法を身につける
- urdfの基本を身につける
- センシングした値をプログラムで扱えるようにする



#### 前提条件、知識

- ROSのインストール
- 基本的なlinuxのコマンドや操作
- 基本的なC++のプログラミング
- 開発に必要なパッケージのインストールについては、本資料では説明を省略する。



#### 内容

1. Gazeboとは
2. Gazeboで簡単なworldを作る
3. urdf形式のロボットモデルを作る
4. Gazebo上でセンシングした値を取得するノードを作る



## 1 Gazeboの操作に慣れる

### 1-1 Gazeboとは

Gazeboとは、物理シミュレーションエンジンを搭載したシミュレーションソフトウェアのことである。ロボティクスへの利用のニーズから生まれたソフトウェアであるため、物理シミュレータの中でもロボット向けの機能が豊富である。物理エンジンとしては、デフォルトの状態ではODE(Open Dynamics Engine)が搭載されている。

ROSとは独立したソフトウェアであるが、ROSと連携させて使うプラグインが公式で多数開発されており、シミュレーションしている情報を直接ROSのトピックとして受け取ったり、逆にROSのトピックを与えることでGazebo内に司令を与えたりすることができる。



### 1-2 起動

インストール後、ターミナルでコマンドを打つことで起動できる。

```bash
gazebo
```

初回起動には時間がかかる。また、`Error in REST request.`という内容のエラーが出た場合は、[この動画](https://www.youtube.com/watch?v=ftDz_EVoatw)のようにして解決する。

ROSでGazeboを扱うパッケージとして、`gazebo_ros`がある。これを用いてROSで起動する場合は、以下のようにする。

```bash
roslaunch gazebo_ros empty_world.launch
```

`roscd gazebo_ros`とすると、システムにインストールされたこのパッケージの場所に移動することができる。launchファイルを見てみると、デフォルトで様々なworldが起動できるようになっていることがわかる。第1回の内容を応用すれば、これらを他のパッケージから利用することも可能である。余裕があれば、launchファイルを直接見てみると理解が深まるだろう。



### 1-3 ウインドウの見方

ウインドウの下部に、時間の再生、一時停止ボタン、時間経過、シミュレーションと実時間の進み方の違いを示す係数(Real Time Factor: 1より小さいと処理が重いためにシミュレーション時間が実時間よりも遅く流れていることを示す。)などが表示されている。

他の部分は、使いながら慣れていくと良い。左上には他の多くのソフトウェアと同じようなツールバーがある。



## 2 Gazeboで簡単なworldを作る

この章は、web上の情報が丁寧にされているので、本資料での説明は簡潔にする。実際に動かしてみることが理解に向けて重要である。

### 2-1 モデルを置いてみる

[公式によるチュートリアル](http://gazebosim.org/tutorials?tut=build_world&cat=build_world)



以下のようなことを試してみると良いだろう。

- 単純なボールや球を置いてみる

- 単純なオブジェクトを動かしてみる

- insertタブから予め定義されているオブジェクトを配置してみる

  

### 2-2 環境を作ってみる

GazeboのBuilding Editorという機能を用いて、建物を作ることができる。

- [公式によるチュートリアル](http://gazebosim.org/tutorials?tut=building_editor&cat=build_world)
- [公式による作成動画](https://www.youtube.com/watch?v=7McYSJFAqlU)

前節の内容と異なり、一度Building Editorから出るとその内容は変更できなくなるので注意すること。

Building Editorから出てモデルを配置したあとに、worldを保存する。



### 2-3 保存と読み込み

左上のツールバーの「File」->「Save world As」の項目を選ぶと、現在のworldをファイルに保存することができる。

拡張子は`.sdf`または`.world`が良いだろう。

gazeboでそのworldを開くには、以下のように引数で与える

```bash
gazebo [world_file]
```

ROSを介してworldを読み込む際は、launchファイルの引数で指定する。以下のlaunchファイルの中身を見て確認してほしい。

```bash
roslaunch gazebo_ros empty_world.launch world_name:=[world_file_path]
```

自作したパッケージからでも、以下の例のように起動できる

```bash
roslaunch nakbot_ros_sim simple_gazebo.launch world_name:=[world_file_path]
```





## 3 urdf形式のロボットモデルを作る

ここでは、urdf形式（Unified Robotic Description Format）を用いてモデルを作る方法を示す。

### 3-1 urdf形式の基本

urdfファイルはxmlを拡張した形式で記述する。ロボットを「link」と「joint」ごとに分け、それらを交互につなげることでモデルを表現する。

一般的なxml形式について補足すると、xml形式は木構造（あるタグの中にタグを記述した入れ込んだ構造）にすることができる。また、`<`や`/>`など、タグの閉じ方のミスによってエラーを起こす初心者が多いので、なれないうちは特に注意すると良い。

#### 基本

単純な箱型のオブジェクトを表示させるurdfファイル

[1_simple_box.urdf](../urdf/tutorial/1_simple_box.urdf)

それぞれの要素についての説明は、[urdfフォーマットに関する仕様](http://wiki.ros.org/urdf/XML)に詳しく記載されている。サンプルやネット上のものを中心に、登場するものからどんなものがあるか覚えていくとよい。今回用いている要素についての説明は3-3に示している。



udfファイルがきちんとしたフォーマット（木構造）になっているかどうかをチェックするには、以下のコマンドを用いる。コマンドがない場合はインストールする。

```bash
check_urdf 1_box.urdf`
```

urdf上で表示させてみるには、`urdf_tutorial`パッケージにちょうどよいlaunchファイルがあるのでそれを利用する。パッケージをインストールしていない場合はインストールするか、このリポジトリにあるほぼ同様のものを用いる。

```bash
roslaunch urdf_tutorial display.launch model:=1_box.urdf

# 本リポジトリにあるほぼ同じ起動コマンド
roslaunch nakbot_ros_sim display.launch model:=1_box.urdf
```



#### 情報を追加する

基本的な箱型のオブジェクトに、もう少し要素を追加してみる。

[2_box.urdf](../urdf/tutorial/2_box.urdf)

衝突に関する属性、urdf上での表示色に関する設定が追加されている。

rvizの「Collision Enabled」のチェックを入れると、衝突を判定する領域の表示ができる。





#### 車輪を追加してみる

移動ロボットらしくするために、車輪を追加してみる。

[3_box_with_wheel.urdf](../urdf/tutorial/3_box_with_wheel.urdf)

`<link>`が複数になり、`<joint>`要素が追加されている。jointはlink同士をつなぐ役割をもち、2つのlinkが固定されているのか、ある可動域で動くのかどうかを定義する。

```xml
<!-- 自由に動けるjointの例 -->
<joint name="right_wheel_joint" type="continuous">`
    <!-- ~いろいろな定義~ -->
</joint>

<!-- 固定されたjointの例 -->
 <joint name="base_joint" type="fixed">
    <!-- ~いろいろな定義~ -->
 </joint>
```

rvizの設定でtfを表示させるようにすると、構造を可視化することができる。rvizのメニューからvisual要素などを半透明にすることで、内部の様子まで確認することができる。



#### センサを追加してみる

Gazeboでセンサからの入力を取得したいので、センサの部分を追加する。

[4_box_with_sensor.urdf](../urdf/tutorial/4_box_with_sensor.urdf)

このあたりから構造が複雑になってくるので、予め紙に書いたり他のツールを用いたりするなどして幾何関係を整理してから記述する必要が出てくるだろう。



### 3-2 Gazeboで必要な追加要素

gazeboでシミュレーションを行うためには、これまでの記述に加えて、シミュレーションに必要な情報を定義する必要がある。

[5_simple_robot.urdf](../urdf/tutorial/5_simple_robot.urdf)

linkの定義の中に`<inertia>`要素が加わり、linkとjointの定義の後に`<gazebo>`要素が加わっている。`<gazebo>`はそれまでに定義されたurdfのlink要素を参照する形で定義する。ここではカメラに関するlinkのみ参照しているが、各linkごとに`<gazebo>`要素を追加し、摩擦係数などを設定することができる。詳細は[Gazebo公式: Gazeboシミュレーションで必要なurdf要素について](http://gazebosim.org/tutorials/?tut=ros_urdf)を参照。



なお、車輪をGazebo上で動かせるようにするためには、ここで記述したものに加えて、車輪を構成する`<joiint>`要素に対してさらなる記述をする必要がある。記述例に関しては、本リポジトリの`my_robo.xacro`にある`differential_drive_controller`に関する部分を参照してほしい。



Gazebo上でurdfで記述したロボットを動かすためのlaunchファイルを用意してあるので、以下のようにして起動する。

```bash
roslaunch nakbot_ros_sim gazebo.launch model:=[urdf_file_path]
```



### 3-3 今回用いたurdf要素

- `<robot>`...ロボット全体の定義
  - `<link>`...リンクの定義
    - `<visual>`...見える部分についての定義。後述するように、衝突判定を計算する境界とは異なる。
      - `<geometry>`...幾何情報の記述。ここでは箱を定義しているが、球や円柱などのオブジェクトを使うこともできる。
      - `<origin>`...基準となる座標からの座標変換
      - `<material>`...visual属性のマテリアルについての設定
    - `<collision>`...衝突判定を計算する部分についての定義。urdfの記述においては必須ではないが、gazeboでシミュレーションを行う上では必須である。
      - `<geometry>`
      - `<origin>`
    - `<inertial>`...物理シミュレーションを行うため、運動に関する情報を定義する。Gazeboでシミュレーションを行う際に必須になる。
      - `<origin>`
      - `<mass>`...質量[kg]
      - `<inertia>`...慣性モーメント[kg*m^2]
  - `<joint>`...ジョイントの定義
    - `<parent>`...親（urdfの木構造のトップに近いほう）のリンクの名前
    - `<child>`...子のリンクの名前
    - `<origin>`...親のリンクの原点からこのjointまでの座標変換







### 3-4 その他

- ここでは紹介しなかったが、[GazeboのModel Editor](http://gazebosim.org/tutorials?tut=model_editor&cat=model_editor_top)を用いてモデルを作成する方法もある。
- ロボットが複雑になってくると、簡単な計算をファイル中に埋め込みたい(定数を定義して数値の書き換えを最小限にしたい)、似たような記述（右の車輪と左の車輪など）を減らしたい、複数ファイルに分散させてモデルの管理を楽にしたい、といった要求が出てくる。urdfを拡張してマクロに対応したxacro形式では、これらに対応することができる。`nakbot_ros_sim`パッケージの`my_robo.xacro`では、この機能を利用してモデルが書かれている。







## 4 Gazebo上でセンシングした値を取得するノードを作る

Gazebo上でシミュレートされたものを取得するといっても、特別必要なことがあるわけではなく、通常のトピック通信と同じようにデータをやりとりすることができる。つまり、[第1回のチュートリアル](https://github.com/sskitajima/ros_tutorial)で画像データをサブスクライブするノードと同じ処理で、画像を受け取ることができる。

第1回の復習も兼ねて、前回作成したコードと同じ構造で、subscribeした画像をその場で表示させるようにしたプログラムを用意している。ただし、第1回はトピック名をコードに直接記述したが、今回はroslaunchの引数タグを用いて記述するようにしている。

```bash
roslaunch nakbot_ros_sim image_subscriber.launch
```





## 5 補足など

### 5-1 参考資料

- [Gazebo公式HP](http://gazebosim.org/)
- [Gazebo公式HPのチュートリアル](http://gazebosim.org/tutorials)
  - 様々な情報がリンクされているので、必要なページを参照するとよい。
- [Gazebo公式: roslaunch](http://gazebosim.org/tutorials?tut=ros_roslaunch&cat=connect_ros)
- [Gazebo公式: Gazeboシミュレーションで必要なurdf要素について](http://gazebosim.org/tutorials/?tut=ros_urdf)
- [ROS公式のurdfに関するチュートリアル](http://wiki.ros.org/ja/urdf/Tutorials)
- [urdfフォーマットに関する仕様](http://wiki.ros.org/urdf/XML)
- [ROS公式: xacro](http://wiki.ros.org/xacro)
- [Gazebo公式のurdfに関するチュートリアル](http://gazebosim.org/tutorials?tut=ros_urdf&cat=connect_ros)

