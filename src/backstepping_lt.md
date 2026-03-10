Dựa trên các nguồn tài liệu, cơ chế điều khiển **Backstepping Thích nghi (Adaptive Backstepping)** được thiết kế để ổn định phương tiện bay (UAV) và điều khiển cánh tay robot bám sát quỹ đạo mong muốn, đồng thời tự động bù trừ các nhiễu loạn chưa biết (như sự thay đổi khối lượng và tác động động lực học chéo giữa UAV và tay máy).

Dưới đây là tổng hợp các công thức toán học cốt lõi của cơ chế này và giải thích chi tiết các tham số:

### 1. Cơ chế Backstepping Tích phân Thích nghi cho UAV (Trục Z - Độ cao)
Mục tiêu là điều khiển lực đẩy tổng $U_1$ để UAV bám sát độ cao mục tiêu, đi kèm thuật toán tự động cập nhật khối lượng ước lượng $\hat{m}$ do UAV phải mang theo tay máy và vật nặng.

*   **Các biến sai số:**
    *   $e_5 = z - z_{des}$: Sai số vị trí độ cao.
    *   $J_5 = \int_0^t e_5(\tau)d\tau$: Sai số tích phân của độ cao,.
    *   $e_6 = \dot{z} - \dot{z}_{des} + K_{5i}J_5 + K_{5p}e_5$: Sai số vận tốc ảo (bước thứ 2 của backstepping).
*   **Công thức tính lực đẩy ($U_1$):**
    $$U_1 = \frac{\hat{m}}{\cos(\phi)\cos(\theta)} \Big(g - K_{5i}e_5 - K_{5p}(e_6 - K_{5i}J_5 - K_{5p}e_5) - e_5 - K_{5d}e_6\Big)$$
*   **Luật cập nhật khối lượng ước lượng ($\dot{\hat{m}}$):**
    $$\dot{\hat{m}} = -c_z e_6 \Big(g - K_{5i}e_5 + K_{5p}(e_6 - K_{5i}J_5 - K_{5p}e_5) - e_5 - K_{5d}e_6\Big)$$

**Giải thích tham số:**
*   $z, z_{des}, \dot{z}, \dot{z}_{des}$: Vị trí và vận tốc độ cao thực tế và mục tiêu,.
*   $\phi, \theta$: Góc Roll và góc Pitch thực tế của UAV.
*   $g$: Gia tốc trọng trường,.
*   $\hat{m}$: Khối lượng ước lượng của toàn hệ thống (UAV + tay máy).
*   $K_{5p}, K_{5i}, K_{5d}$: Lần lượt là các hệ số khuếch đại tỉ lệ, tích phân và vi phân (mang giá trị dương) để thiết kế tốc độ hội tụ của bộ điều khiển trục Z.
*   $c_z$: Hệ số thích nghi (hằng số dương) quyết định tốc độ cập nhật/học của khối lượng ước lượng.

---

### 2. Cơ chế Backstepping Thích nghi điều khiển Góc Tư thế UAV (Attitude Control)
Mục tiêu là tính toán các mô-men xoắn $U_2, U_3, U_4$ để điều khiển các góc Roll, Pitch, Yaw, đồng thời bù trừ mô-men nhiễu từ tay máy robot. Các công thức dựa trên các biến sai số tương tự ($e_7 \dots e_{12}$) cho các góc tư thế:

*   **Công thức tính Mô-men Roll ($U_2$):**
    $$U_2 = \frac{I_{xx}}{l_x} \left( -K_{7i}e_7 - K_{7p}(e_8 - K_{7i}J_7 - K_{7p}e_7) + \frac{\dot{\theta}\dot{\psi}(I_{zz}-I_{yy})}{I_{xx}} - e_7 - K_{7d}e_8 - \frac{\hat{n}_{0x}}{I_{xx}} - \frac{g_x}{I_{xx}} \right)$$
*   **Công thức tính Mô-men Pitch ($U_3$):**
    $$U_3 = \frac{I_{yy}}{l_y} \left( -K_{9i}e_9 - K_{9p}(e_{10} - K_{9i}J_9 - K_{9p}e_9) + \frac{\dot{\phi}\dot{\psi}(I_{xx}-I_{zz})}{I_{yy}} - e_9 - K_{9d}e_{10} - \frac{\hat{n}_{0y}}{I_{yy}} - \frac{g_y}{I_{yy}} \right)$$
*   **Công thức tính Mô-men Yaw ($U_4$):**
    $$U_4 = I_{zz} \left( -K_{11i}e_{11} - K_{11p}(e_{12} - K_{11i}J_{11} - K_{11p}e_{11}) + \frac{\dot{\phi}\dot{\theta}(I_{yy}-I_{xx})}{I_{zz}} - e_{11} - K_{11d}e_{12} - \frac{\hat{n}_{0z}}{I_{zz}} - \frac{g_z}{I_{zz}} \right)$$

**Giải thích tham số:**
*   $I_{xx}, I_{yy}, I_{zz}$: Các mô-men quán tính của UAV quanh trục X, Y, Z,.
*   $l_x, l_y$: Chiều dài tay đòn từ tâm UAV đến các trục rotor.
*   $K_7, K_9, K_{11}$ (với các hậu tố $p, i, d$): Các hệ số khuếch đại thiết kế dương tương ứng cho bộ điều khiển trục Roll, Pitch và Yaw,.
*   $\hat{n}_{0x}, \hat{n}_{0y}, \hat{n}_{0z}$: Mô-men nhiễu ước lượng tác động lên đế UAV sinh ra do tương tác động lực học với tay máy,,.
*   $g_x, g_y, g_z$: Các mô-men tác động lên UAV sinh ra do sự dịch chuyển trọng tâm (Center of Gravity displacement) của cánh tay robot.
*   $\frac{\dot{\theta}\dot{\psi}(I_{zz}-I_{yy})}{I_{xx}} \dots$: Các thành phần bù trừ hiệu ứng con quay hồi chuyển và lực Coriolis,.

---

### 3. Cơ chế Backstepping Thích nghi cho Cánh tay Robot (Manipulator)
Mục tiêu là tính toán lực/mô-men $\tau_i$ cấp cho từng khớp tay máy sao cho robot bám sát quỹ đạo góc, đồng thời bù trừ sự chuyển động của phần đế (UAV) đang bay.

*   **Các biến sai số khớp:** (Với $i = 1, 2, 3, 4, 5, 6$)
    *   $e_{i1} = q_i - q_{i,des}$: Sai số vị trí góc khớp.
    *   $e_{i2} = \dot{q}_i - \dot{q}_{i,des} + K_{pi}e_{i1}$: Sai số vận tốc ảo.
*   **Mảng gia tốc hội tụ mục tiêu ($Q_{dd}$):**
    Tập hợp gia tốc cần thiết để triệt tiêu sai số của các khớp:
    $$Q_{dd} = \begin{bmatrix}
    -K_{p1}(e_{12} - K_{p1}e_{11}) - e_{11} - K_{d1}e_{12} \\
    -K_{p2}(e_{22} - K_{p2}e_{21}) - e_{21} - K_{d2}e_{22} \\
    -K_{p3}(e_{32} - K_{p3}e_{31}) - e_{31} - K_{d3}e_{32} \\
    -K_{p4}(e_{42} - K_{p4}e_{41}) - e_{41} - K_{d4}e_{42} \\
    -K_{p5}(e_{52} - K_{p5}e_{51}) - e_{51} - K_{d5}e_{52} \\
    -K_{p6}(e_{62} - K_{p6}e_{61}) - e_{61} - K_{d6}e_{62} \\
    \end{bmatrix}$$
*   **Công thức tính mô-men xoắn bù cho các khớp ($\tau$):**
    $$\begin{bmatrix} \tau_1 \\ \tau_2 \\ \tau_3 \\ \tau_4 \\ \tau_5 \\ \tau_6 \end{bmatrix} = (D^{-1}_{7:12,7:12})^{-1} \left[ Q_{dd} - D^{-1}_{7:12,1:6}(\tau_{1:6} - H_{1:6}) \right] + H_{7:12}$$

**Giải thích tham số:**
*   $q_i, \dot{q}_i$: Góc khớp và vận tốc góc khớp hiện tại của robot.
*   $K_{pi}, K_{di}$: Hệ số khuếch đại thiết kế tỉ lệ và vi phân cho mỗi khớp.
*   $D, H$: Lần lượt là ma trận quán tính đối xứng của toàn bộ hệ thống và véc-tơ chứa lực ly tâm, Coriolis, trọng lực.
*   $D^{-1}_{a:b, c:d}$: Là ma trận con (submatrix) được trích xuất từ ma trận nghịch đảo $D^{-1}$ (từ hàng $a$ đến $b$, cột $c$ đến $d$). Chỉ số 1:6 đại diện cho các trạng thái của UAV, chỉ số 7:9 đại diện cho các trạng thái tương tác nội bộ của 3 khớp robot,.
*   $\tau_{1:6}$ và $H_{1:6}$: Tín hiệu đầu vào và các thành phần động lực học tương ứng với 6 bậc tự do của bản thân UAV.
*   $H_{7:12}$: Các thành phần phi tuyến và trọng lực tác động lên tay máy robot.