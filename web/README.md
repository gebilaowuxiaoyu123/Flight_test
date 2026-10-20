# Web 遥控器前端

P5 实现。计划使用**无构建步骤**的原生 HTML/CSS/JS，由 `flight_ui` 的 FastAPI 直接托管，避免引入 Node 工具链。

## 计划文件

```
web/
├── index.html      # 布局：HUD + 双摇杆 + 按钮
├── style.css       # 深色、适合手机横屏
└── app.js          # WebSocket、摇杆、触摸/鼠标、Gamepad API
```

## 交互

- 左摇杆：上下油门、左右偏航
- 右摇杆：上下俯仰、左右横滚
- 按钮：ARM / DISARM / POSCTL / KILL
- Deadman：按住才发送有效摇杆
- HUD：ARMED、模式、高度、电量、连接状态

## 协议

见 [docs/04-rc-ui.md](../docs/04-rc-ui.md)。

## 注意

- 手机上禁用页面滚动/缩放，避免摇杆操作时页面跟着动。
- 松开触摸/鼠标必须回中。
- WebSocket 断开时显示红色状态，并停止发送。
