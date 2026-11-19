# shape 截面形狀計算工具

## 1. 空心矩形截面 (HollowRectangleSection)

### 1.1 幾何參數

```python
def __init__(self, width, height, thickness=0):
    self.b = width          # 外寬
    self.h = height         # 外高
    self.t = thickness      # 壁厚
    
    # 內部尺寸
    self.bi = max(0, self.b - 2 * self.t)  # 內寬
    self.hi = max(0, self.h - 2 * self.t)  # 內高
```

**幾何關係**：
- 外矩形：$b \times h$
- 內矩形：$b_i \times h_i$
- 壁厚：$t$

### 1.2 面積計算

**公式**：

$$A = b \cdot h - b_i \cdot h_i$$

**推導**：
- 總面積 = 外面積 - 內面積
- 當 $t = 0$ 時：$A = b \cdot h$（實心矩形）

### 1.3 X軸慣性矩 (Ix)

**公式**：

$$I_x = \frac{b h^3}{12} - \frac{b_i h_i^3}{12}$$

**推導**：
- 外矩形對X軸慣性矩：$I_{x,\text{outer}} = \frac{b h^3}{12}$
- 內矩形對X軸慣性矩：$I_{x,\text{inner}} = \frac{b_i h_i^3}{12}$
- 空心矩形慣性矩：$I_x = I_{x,\text{outer}} - I_{x,\text{inner}}$

### 1.4 Y軸慣性矩 (Iy)

**公式**：

$$I_y = \frac{h b^3}{12} - \frac{h_i b_i^3}{12}$$

**推導**：
- 外矩形對Y軸慣性矩：$I_{y,\text{outer}} = \frac{h b^3}{12}$
- 內矩形對Y軸慣性矩：$I_{y,\text{inner}} = \frac{h_i b_i^3}{12}$
- 空心矩形慣性矩：$I_y = I_{y,\text{outer}} - I_{y,\text{inner}}$

### 1.5 極慣性矩 (J)

**公式**：

$$J = I_x + I_y$$

**推導**：
- 根據定義，極慣性矩等於兩個主軸慣性矩之和
- 用於扭轉問題

### 1.6 X軸截面模量 (Zx)

**公式**：

$$Z_x = \frac{I_x}{h/2} = \frac{2 I_x}{h}$$

**推導**：
- 截面模量定義：$Z = \frac{I}{c}$，其中 $c$ 為截面至中性軸的最大距離
- 對於矩形截面，$c = h/2$
- 用於彎曲應力計算：$\sigma = \frac{M}{Z}$

### 1.7 Y軸截面模量 (Zy)

**公式**：

$$Z_y = \frac{I_y}{b/2} = \frac{2 I_y}{b}$$

**推導**：
- 類似於 $Z_x$，但對Y軸
- $c = b/2$
### 1.8 極截面模量 (Zp)

**公式**：

$$Z_p = \frac{J}{r}$$

其中 $r = \sqrt{(b/2)^2 + (h/2)^2}$

**推導**：
- 極截面模量用於扭轉應力計算
- $r$ 為從形心至角點的距離（最大距離）
- 扭轉應力：$\tau = \frac{T}{Z_p}$


## 2. 空心圓形截面 (HollowCircleSection)

### 2.1 幾何參數

```python
def __init__(self, outer_radius, thickness=0):
    self.ro = outer_radius    # 外徑
    self.t = thickness        # 壁厚
    
    self.ri = max(0, self.ro - self.t)  # 內徑
```

**幾何關係**：
- 外半徑：$r_o$
- 內半徑：$r_i = r_o - t$
- 當 $t = 0$ 時：$r_i = 0$（實心圓）

### 2.2 面積計算

**公式**：

$$A = \pi (r_o^2 - r_i^2)$$

**推導**：
- 總面積 = 外圓面積 - 內圓面積
- 外圓面積：$\pi r_o^2$
- 內圓面積：$\pi r_i^2$

### 2.3 X軸慣性矩 (Ix)

**公式**：

$$I_x = \frac{\pi}{4} (r_o^4 - r_i^4)$$

**推導**：
- 實心圓對任意直徑的慣性矩：$I = \frac{\pi r^4}{4}$
- 空心圓慣性矩：$I_x = I_{x,\text{outer}} - I_{x,\text{inner}}$
- 對於圓形截面，$I_x = I_y$



### 2.4 Y軸慣性矩 (Iy)

**公式**：

$$I_y = I_x = \frac{\pi}{4} (r_o^4 - r_i^4)$$

**推導**：
- 由於圓形的對稱性，$I_x = I_y$


### 2.5 極慣性矩 (J)

**公式**：

$$J = \frac{\pi}{2} (r_o^4 - r_i^4)$$

**推導**：
- 對於圓形截面：$J = 2 I_x = 2 \cdot \frac{\pi}{4} (r_o^4 - r_i^4) = \frac{\pi}{2} (r_o^4 - r_i^4)$
- 極慣性矩用於扭轉分析


### 2.6 X軸截面模量 (Zx)

**公式**：

$$Z_x = \frac{I_x}{r_o}$$

**推導**：
- 對於圓形截面，最大距離 $c = r_o$
- 截面模量：$Z_x = \frac{I_x}{c} = \frac{I_x}{r_o}$


### 2.7 Y軸截面模量 (Zy)

**公式**：

$$Z_y = Z_x = \frac{I_x}{r_o}$$

**推導**：
- 由於對稱性，$Z_x = Z_y$


### 2.8 極截面模量 (Zp)

**公式**：

$$Z_p = \frac{J}{r_o}$$

**推導**：
- 極截面模量：$Z_p = \frac{J}{c}$，其中 $c$ 為最大距離
- 對於圓形，$c = r_o$


## 3. 公式整合

### 3.1 實心矩形

對於實心矩形 ($t = 0$)：

- 面積：$A = b \cdot h$
- 慣性矩：$I_x = \frac{b h^3}{12}, \quad I_y = \frac{h b^3}{12}$
- 截面模量：$Z_x = \frac{b h^2}{6}, \quad Z_y = \frac{h b^2}{6}$

### 3.2 實心圓

對於實心圓 ($t = 0$)：

- 面積：$A = \pi r_o^2$
- 慣性矩：$I_x = I_y = \frac{\pi r_o^4}{4}$
- 極慣性矩：$J = \frac{\pi r_o^4}{2}$
- 截面模量：$Z_x = Z_y = \frac{\pi r_o^3}{4}$
- 極截面模量：$Z_p = \frac{\pi r_o^3}{2}$

### 3.3 薄壁近似

當壁厚 $t \ll r_o$ 時：

- 圓管面積：$A \approx 2 \pi r_o t$
- 圓管慣性矩：$I_x \approx \frac{\pi r_o^3 t}{2}$
