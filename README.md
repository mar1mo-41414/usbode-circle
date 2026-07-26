# USBODE: USB 光学ドライブエミュレータ (BLE 対応フォーク)

[English README →](README-EN.md)

Raspberry Pi Zero W / Zero 2 W などを、USB 接続の仮想 CD-ROM ドライブに変える
[danifunker/usbode-circle](https://github.com/danifunker/usbode-circle) のフォークです。

このフォークでは **BLE シリアル (Nordic UART Service) によるディスクイメージ切り替え**を追加しています。
Wi-Fi ネットワークが無くても、スマホや PC の BLE ターミナルアプリから ISO の入れ替えができます。

オリジナルの機能 (Web UI、FTP、OLED/LCD HAT、CD オーディオなど) はそのまま残っています。
詳細なハードウェア要件・セットアップ手順は [README-EN.md](README-EN.md) を参照してください。

## BLE シリアルでのディスク切り替え

### 対応ボード

オンボード Bluetooth を持つモデルで動作します:

| モデル | BT チップ | ファームウェア |
|---|---|---|
| Pi Zero W | BCM43438 | `BCM43430A1.hcd` |
| Pi Zero 2 W | BCM43430A1 | `BCM43430A1.hcd` |
| Pi 3A+ / 3B+ | BCM4345 | `BCM4345C0.hcd` |
| Pi 4 / 400 / CM4 | BCM4345 | `BCM4345C0.hcd` |

`.hcd` ファームウェアはビルド時に自動ダウンロードされ、SD カードの `firmware/` に配置されます。
どのファイルを使うかは起動時にチップ自身へ型番を問い合わせて自動選択します
(HCI Read_Local_Name の応答からファイル名を決定)。
(無印 Pi Zero など BT 非搭載モデルでは BLE サービスは自動的に無効になります)

> **動作確認済み**: Raspberry Pi Zero 2 W + iPhone の Bluefruit Connect (UART) で
> `help` / `list` / `mount` の動作を確認しています (2026-07-23)。

### 使い方

1. スマホに BLE シリアルターミナルアプリを入れる
   (例: iOS/Android の「Serial Bluetooth Terminal」「nRF Connect」「Bluefruit Connect」など、
   Nordic UART Service 対応のもの)
2. `USBODE` という名前のデバイスに接続する
3. TX キャラクタリスティックの Notify を有効にして、コマンドを送信する

### コマンド

```
help          このヘルプを表示
info          現在のイメージとバージョンを表示
list [page]   ディスクイメージ一覧 (15 件/ページ)
mount <番号>  list の番号でイメージをマウント
mount <名前>  ファイル名 (相対パス) でマウント
reboot        再起動
shutdown      シャットダウン
```

例:

```
> list
0: DOS_GAMES/doom.iso
1: image.iso *
2: win98se.iso
page 1/1 (3 images)
OK
> mount 0
OK mounting DOS_GAMES/doom.iso
```

### 設定

`config.txt` の `[usbode]` セクションで無効化できます (デフォルトは有効):

```
bleenabled=0
```

注意: BLE はオンボード BT チップとの通信に PL011 (UART0) を使用します。
`cmdline.txt` で `logdev=ttyS1` (シリアルログ) を指定している場合、
BLE サービスは競合を避けるため自動的に無効になります。

## ⚠️ セキュリティ上の注意

**この BLE コンソールは認証・ペアリングを一切要求しません。** 電波の届く範囲に
いる人なら誰でも、接続してディスク一覧の閲覧・マウント中の ISO の入れ替え・
本体のシャットダウンができてしまいます。

- 人が多い場所 (イベント会場など) では十分注意してください。不特定多数から
  勝手に操作される可能性があります。
- 使わないときは `config.txt` に `bleenabled=0` を書いて無効化してください。
- ペアリング必須化などのアクセス制御は現状**実装されていません**。信頼できない
  環境での利用は自己責任でお願いします。

> このフォークは個人利用を想定した実験的な実装です。認証を追加するまでは、
> 自宅など管理された環境での利用を推奨します。

## 実機依存・リアルタイム性についての注意

USBODE の本分はホストの SCSI タイムアウト内に応答し続けることです。Circle は
協調的スケジューリング (プリエンプションなし) なので、BLE 処理も同じループを
共有します。本フォークは Pi Zero 2 W で動作しますが、**BLE トラフィック下での
USB ガジェットのレイテンシは実測していません**。ビンテージ機で重い I/O 中に
BLE を多用すると、理論上は SCSI 応答が遅れて I/O エラーやディスク脱落を招く
可能性があります。実運用前にご自身の環境で確認してください。

## ビルド

基本手順はオリジナルと同じです ([BUILD.md](BUILD.md) 参照):

```bash
# 全アーキテクチャ (32bit) ビルド
make multi-arch

# 単一アーキテクチャ (例: Pi Zero W)
make RASPPI=1 dist-single
```

SD カードに焼ける `.img` の作成:

```bash
# Linux
make image-single

# macOS (このフォークで追加)
scripts/create-img-macos.sh -s dist -o imgout -n usbode.img
```

生成された `.img` を Raspberry Pi Imager などで SD カードに書き込み、
`bootfs` パーティションの `wpa_supplicant.conf` を編集 (Wi-Fi を使う場合) すれば完了です。
BLE だけで使う場合は Wi-Fi 設定は不要です。

## 実装メモ

- `addon/bleservice/` — BLE スタック本体
  - `btuarttransport.cpp` — PL011 (GPIO32/33 ALT3) 経由の HCI UART トランスポート
    (Circle が 2020 年に削除した Bluetooth サポートをベースに ACL 対応を追加)
  - `bleservice.cpp` — .hcd ファームウェアロード、LE アドバタイズ、
    最小 ATT サーバー (GAP + Nordic UART Service)、コマンドシェル
- イメージ切り替えは Web UI と同じ `SCSITBService::SetNextCDByName()` を使用

## ライセンス

オリジナルと同じ GPLv3 です。[LICENSE](LICENSE) を参照してください。
