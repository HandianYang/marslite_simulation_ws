# TM5-Series Robotic Arm Kinematics Derivation

## Denavit Hartenberg (DH) parameters

The DH parameters for TM5_700 robotic arm:
| Links | $\theta$ $(\degree)$ | $\alpha$ $(\degree)$ | $a$ $(mm)$ | $d$ $(mm)$ | $Offset$ $(\degree)$ |
| -: | ---------: | -----------: | ------: | -------: | -----------: |
|  1 | $\Theta_1$ | $-90\degree$ |     $0$ |  $145.1$ |   $0\degree$ |
|  2 | $\Theta_2$ |   $0\degree$ |   $329$ |      $0$ | $-90\degree$ |
|  3 | $\Theta_3$ |   $0\degree$ | $311.5$ |      $0$ |   $0\degree$ |
|  4 | $\Theta_4$ |  $90\degree$ |     $0$ | $-122.2$ |  $90\degree$ |
|  5 | $\Theta_5$ |  $90\degree$ |     $0$ |    $106$ |   $0\degree$ |
|  6 | $\Theta_6$ |   $0\degree$ |     $0$ |  $114.4$ |   $0\degree$ |

For DH parameters of other kinds of TM5-Series robotic arm, please check the class constructor definition in  `tm_kinematics.cpp`.

## Forward kinematics

The transformation matrix between coordinates frame `n-1` and `n`:
$$
\begin{equation}
    A_n =
    \begin{bmatrix*}[r]
        \cos(\theta_n) & -\sin(\theta_n)\cos(\alpha_n) & \sin(\theta_n)\sin(\alpha_n) & a_n\cos(\theta_n) \\ 
        \sin(\theta_n) & \cos(\theta_n)\cos(\alpha_n) & -\cos(\theta_n)\sin(\alpha_n) & a_n\sin(\theta_n) \\ 
        0 & \sin(\alpha_n) & \cos(\alpha_n) & d_n \\
        0 & 0 & 0 & 1
    \end{bmatrix*}
\end{equation}
$$
where $\theta_n$ represents the **joint angles with offset**. For example, if the angle of joint $2$ is $-42\degree$, then
$\theta_2 = -42\degree - 90\degree = -132\degree$.

Consequently, the transformation matrices of 6 links:
$$
\begin{equation}
    A_1 =
    \begin{bmatrix*}[r]
        \cos(\theta_1) & 0 & -\sin(\theta_1) & 0 \\ 
        \sin(\theta_1) & 0 &  \cos(\theta_1) & 0 \\ 
        0 & -1 & 0 & d_1 \\
        0 & 0 & 0 & 1
    \end{bmatrix*}
\end{equation}
$$

$$
\begin{equation}
    A_2 =
    \begin{bmatrix*}[r]
        \cos(\theta_2) & -\sin(\theta_2) & 0 & a_2\cos(\theta_2) \\ 
        \sin(\theta_2) &  \cos(\theta_2) & 0 & a_2\sin(\theta_2) \\ 
        0 & 0 & 1 & 0 \\
        0 & 0 & 0 & 1
    \end{bmatrix*}
\end{equation}
$$

$$
\begin{equation}
    A_3 =
    \begin{bmatrix*}[r]
        \cos(\theta_3) & -\sin(\theta_3) & 0 & a_3\cos(\theta_3) \\ 
        \sin(\theta_3) &  \cos(\theta_3) & 0 & a_3\sin(\theta_3) \\ 
        0 & 0 & 1 & 0 \\
        0 & 0 & 0 & 1
    \end{bmatrix*}
\end{equation}
$$

$$
\begin{equation}
    A_4 =
    \begin{bmatrix*}[r]
        \cos(\theta_4) & 0 &  \sin(\theta_4) & 0 \\ 
        \sin(\theta_4) & 0 & -\cos(\theta_4) & 0 \\ 
        0 & 1 & 0 & d_4 \\
        0 & 0 & 0 & 1
    \end{bmatrix*}
\end{equation}
$$

$$
\begin{equation}
    A_5 =
    \begin{bmatrix*}[r]
        \cos(\theta_5) & 0 &  \sin(\theta_5) & 0 \\ 
        \sin(\theta_5) & 0 & -\cos(\theta_5) & 0 \\ 
        0 & 1 & 0 & d_5 \\
        0 & 0 & 0 & 1
    \end{bmatrix*}
\end{equation}
$$

$$
\begin{equation}
    A_6 =
    \begin{bmatrix*}[r]
        \cos(\theta_6) & -\sin(\theta_6) & 0 & 0 \\ 
        \sin(\theta_6) &  \cos(\theta_6) & 0 & 0 \\ 
        0 & 0 & 1 & d_6 \\
        0 & 0 & 0 & 1
    \end{bmatrix*}
\end{equation}
$$

The transformation matrix from the `base` frame to the `end-effector` frame:
$$
\begin{equation}
    T_6 = A_1 A_2 A_3 A_4 A_5 A_6 = 
    \begin{bmatrix*}[c]
        n_x & o_x & a_x & p_x \\
        n_y & o_y & a_y & p_y \\
        n_z & o_z & a_z & p_z \\
        0 & 0 & 0 & 1
    \end{bmatrix*}
\end{equation}
$$

## Inverse kinematics

The derivation follows algebraic methods, which is a classic method in inverse kinematics, to obtain the closed-form solutions. If you prefer some modern methods, just ignore this section! 

The process can be split into 3 parts: 
1. Solve $\theta_1$.
2. Solve $\theta_5$, $\theta_6$, and $\theta_P$ based on $\theta_1$. Specifically, $\theta_P = \theta_2 + \theta_3 + \theta_4$.
3. Solve $\theta_2$, $\theta_3$, and $\theta_4$ based on $\theta_1$ and $\theta_P$.

**[Note]** To simplify the equations, we rewrite the trigonometric function notation as follows:
- $S_n = \sin(\theta_n)$
- $C_n = \cos(\theta_n)$
- $S_{abc} = \sin(\theta_a + \theta_b + \theta_c)$
- $C_{abc} = \cos(\theta_a + \theta_b + \theta_c)$

### (1) Solve $\theta_1$

#### Derivation

From $T_6$, we can obtain:
$$
\begin{equation}
    \begin{cases}
        p_x = C_1(a_2C_2 + a_3C_{23} + d_5S_{234} + d_6C_{234}S_5) - S_1(d_4 - d_6C_5) \\
        p_y = S_1(a_2C_2 + a_3C_{23} + d_5S_{234} + d_6C_{234}S_5) + C_1(d_4 - d_6C_5) \\
        p_z = -(a_2S_2 + a_3S_{23} - d_5C_{234} + d_6S_{234}S_5) + d_1 \\
        a_x = C_1C_{234}S_5 + S_1C_5 \\
        a_y = S_1C_{234}S_5 - C_1C_5 \\
        a_z = -S_{234}S_5
    \end{cases}
\end{equation}
$$

Combine these results and obtain:
$$
\begin{equation}
    \begin{cases}
        p_x - d_6a_x & = C_1(a_2C_2 + a_3C_{23} + d_5S_{234}) - d_4S_1 \\
        p_y - d_6a_y & = S_1(a_2C_2 + a_3C_{23} + d_5S_{234}) + d_4C_1 \\
        p_z - d_1 - d_6a_z & = -(a_2S_2 + a_3S_{23} - d_5C_{234})
    \end{cases}
\end{equation}
$$

Here, we take the value of $p_x - d_6a_x$ and $p_y - d_6a_y$, and construct the equation:
$$
\begin{equation}
    C_1(p_x - d_6a_x) - S_1(p_y - d_6a_y) = d_4
\end{equation}
$$

Let $r\sin(\phi) = p_x - d_6a_x$ and $r\cos(\phi) = p_y - d_6a_y$, where $r = \sqrt{(p_x - d_6a_x)^2 + (p_y - d_6a_y)^2}$ and $\phi = \tan^{-1}{(\frac{p_y - d_6a_y}{p_x - d_6a_x})}$. The equation becomes:
$$
\begin{equation}
    \begin{split}
        C_1\sin(\phi) - S_1\cos(\phi) &= \frac{d4}{r} \\
        -\sin(\theta_1 - \phi) &= \frac{d4}{r} \\
        \theta_1 - \phi &= \sin^{-1}(-\frac{d4}{r})
    \end{split}
\end{equation}
$$

From the aspect of convertion between trigonometric functions, we can convert $\sin^{-1}$ to $\tan^{-1}$ (or more precisely, $atan2$) to ensure that the function will not reach to $\pm\infty$:
$$
\begin{equation}
    \begin{split}
        \theta_1 - \phi &= \sin^{-1}(-\frac{d_4}{r}) \\
                        &= \tan^{-1}(\frac{-d_4}{\pm\sqrt{r^2 - {d_4}^2}})
    \end{split}
\end{equation}
$$

Hence, the solution of $\theta_1$ shall be
$$
\begin{equation}
    \begin{split}
        \theta_1 &= \phi + \tan^{-1}(\frac{-d_4}{\pm\sqrt{r^2 - {d_4}^2}}) \\
                 &= \tan^{-1}{(\frac{p_y - d_6a_y}{p_x - d_6a_x})} + \tan^{-1}(\frac{-d_4}{\pm\sqrt{r^2 - {d_4}^2}})
    \end{split}
\end{equation}
$$

Here, we have two different configurations regarding the first joint, so we take two values of $\theta_1$.
$$
\begin{equation}
    \begin{split}
        \theta_1 &= \tan^{-1}{(\frac{p_y - d_6a_y}{p_x - d_6a_x})} + \tan^{-1}(\frac{-d_4}{\sqrt{r^2 - {d_4}^2}}) \\
                 &\text{ or } \tan^{-1}{(\frac{-(p_y - d_6a_y)}{-(p_x - d_6a_x)})} - \tan^{-1}(\frac{-d_4}{\sqrt{r^2 - {d_4}^2}})
    \end{split}
\end{equation}
$$

#### Exception

**If $r < -d_4$**, then the value of $\sin^{-1}(-\frac{d_4}{r})$ does not exist. There shall be **no solutions** in this case.

### (2) Solve $\theta_5$, $\theta_6$, and $\theta_P$

From ${A_1}^{-1}T_6$, we can obtain:
$$
\begin{equation}
    \begin{cases}
         C_1a_x + S_1a_y &= C_{234}S_5 \\
                    -a_z &= S_{234}S_5 \\
        -S_1a_x + C_1a_y &= -C_5 \\
        -S_1n_x + C_1n_y &=  S_5C_6 \\
        -S_1o_x + C_1o_y &= -S_5S_6
    \end{cases}
\end{equation}
$$

The solution of $\lbrace \theta_5, \theta_6, \theta_P \rbrace$ forms a configuration. In general, there are two different configurations regarding $\theta_5$, $\theta_6$, and $\theta_P$, so there shall be two solution sets.
 
#### Derivation on solving $\theta_5$

From the third equation in $(16)$, we could easily obtain $\theta_5$
$$
\begin{equation}
    \begin{split}
        C_5 &= -(-S_1a_x + C_1a_y) \\
        \theta_5 &= \cos^{-1}(S_1a_x - C_1a_y)
    \end{split}
\end{equation}
$$

The solutions of $\theta_5$ of the two different configurations are:
$$
\begin{equation}
    \theta_5 = \cos^{-1}(S_1a_x - C_1a_y) \text{ or } -\cos^{-1}(S_1a_x - C_1a_y)
\end{equation}
$$

#### Derivation on solving $\theta_6$

Dividing the fifth equation by the fourth equation from $(16)$ results in 
$$
\begin{equation}
    \begin{split}
        \frac{-S_5S_6}{S_5C_6} &= \frac{-S_1o_x + C_1o_y}{-S_1n_x + C_1n_y} \\
        \theta_6 &= \tan^{-1}(\frac{-(-S_1o_x + C_1o_y)}{-S_1n_x + C_1n_y})
    \end{split}
\end{equation}
$$

The solutions of $\theta_6$ of the two different configurations are:
$$
\begin{equation}
    \theta_6 = \tan^{-1}(\frac{-(-S_1o_x + C_1o_y)}{-S_1n_x + C_1n_y}) \text{ or } \tan^{-1}(\frac{-S_1o_x + C_1o_y}{-(-S_1n_x + C_1n_y)})
\end{equation}
$$

#### Derivation on solving $\theta_P$

Dividing the first equation by the second equation from $(16)$ results in 
$$
\begin{equation}
    \begin{split}
        \frac{S_{234}S_5}{C_{234}S_5} &= \frac{-a_z}{C_1a_x + S_1a_y} \\
        \theta_P = \theta_{234} &= \tan^{-1}(\frac{-a_z}{C_1a_x + S_1a_y})
    \end{split}
\end{equation}
$$

The solutions of $\theta_P$ of the two different configurations are:
$$
\begin{equation}
    \theta_P = \tan^{-1}(\frac{-a_z}{C_1a_x + S_1a_y}) \text{ or } \tan^{-1}(\frac{-(-a_z)}{-(C_1a_x + S_1a_y)})
\end{equation}
$$

#### Exception

The number of solution sets $\lbrace \theta_5, \theta_6, \theta_P \rbrace$ would **degenerate from two to one if $S_5 \approx 0$**.

For $\theta_5$, $S_5 \approx 0$ if $S_1a_x - C_1a_y \approx 1 \text{ or } -1$.
- If $S_1a_x - C_1a_y \approx 1$, then $\theta_5 = \cos^{-1}(1) = 0$
- If $S_1a_x - C_1a_y \approx -1$, then $\theta_5 = \cos^{-1}(-1) = \pi$

As for $\theta_6$ and $\theta_P$, we calculate ${A_1}^{-1}T_6$ again under the condition "$S_5 \approx 0$". Two cases are considered: $C_5 \approx 1$ and $C_5 \approx -1$.

##### [Case 1] $C_5 \approx 1$

$$
\begin{equation}
    {A_1}^{-1}T_6 =
    \begin{bmatrix*}[r]
        C_{234}C_6 + S_{234}S_6 & -C_{234}S_6 + S_{234}C_6 & 0 & a_2C_2 + a_3C_{23} + d_5S_{234} \\
        S_{234}C_6 - C_{234}S_6 & -S_{234}S_6 - C_{234}C_6 & 0 & a_2S_2 + a_3S_{23} - d_5C_{234} \\
        0 & 0 & -1 & d_4 - d_6 \\
        0 & 0 & 0 & 1
    \end{bmatrix*}
\end{equation}
$$

From ${A_1}^{-1}T_6$, we can obtain
$$
\begin{equation}
    \begin{cases}
        C_1n_x + S_1n_y &= \cos(\theta_{234} - \theta_6) \\
                   -n_z &= \sin(\theta_{234} - \theta_6)
    \end{cases}
\end{equation}
$$

Dividing the second equation by the first equation of $(24)$ results in
$$
\begin{equation}
    \theta_{234} - \theta_6 = \tan^{-1}(\frac{-n_z}{C_1n_x + S_1n_y})
\end{equation}
$$

##### [Case 2] $C_5 \approx -1$

$$
\begin{equation}
    {A_1}^{-1}T_6 =
    \begin{bmatrix*}[r]
        -C_{234}C_6 + S_{234}S_6 & C_{234}S_6 + S_{234}C_6 & 0 & a_2C_2 + a_3C_{23} + d_5S_{234} \\
        -S_{234}C_6 - C_{234}S_6 & S_{234}S_6 - C_{234}C_6 & 0 & a_2S_2 + a_3S_{23} - d_5C_{234} \\
        0 & 0 & 1 & d_4 + d_6 \\
        0 & 0 & 0 & 1
    \end{bmatrix*}
\end{equation}
$$

From ${A_1}^{-1}T_6$, we can obtain
$$
\begin{equation}
    \begin{cases}
        C_1n_x + S_1n_y &= -\cos(\theta_{234} - \theta_6) \\
                   -n_z &= -\sin(\theta_{234} - \theta_6)
    \end{cases}
\end{equation}
$$

Dividing the second equation by the first equation of $(24)$ results in
$$
\begin{equation}
    \theta_{234} - \theta_6 = \tan^{-1}(\frac{-n_z}{C_1n_x + S_1n_y})
\end{equation}
$$

The results from these two cases are the same.

We can choose $\theta_6$ as an arbitrary value, and then $\theta_{234} = \theta_6 + \tan^{-1}(\frac{-n_z}{C_1n_x + S_1n_y})$.



### (2) Solve $\theta_2$, $\theta_3$, and $\theta_4$

We utilize the equation set $(10)$ to solve $\theta_2$, $\theta_3$, and $\theta_4$. By performing some operations, we obtain
$$
\begin{equation}
    \begin{cases}
        C_1(p_x - d_6a_x) + S_1(p_y - d_6a_y) - d_5S_{234} &= a_2C_2 + a_3C_{23} = A \\
        -p_z + d_1 + d_6a_z + d_5C_{234} &= a_2S_2 + a_3S_{23} = B
    \end{cases}
\end{equation}
$$
Here, we denote two equations as $A$ and $B$, respectively.

The solution of $\lbrace \theta_2, \theta_3, \theta_4 \rbrace$ forms a configuration. In general, there are two different configurations regarding $\theta_2$, $\theta_3$, and $\theta_4$, so there shall be two solution sets.

#### Derivation on solving $\theta_3$

We add $A^2$ to $B^2$ and obtain
$$
\begin{equation}
    \begin{split}
        A^2 + B^2 &= {a_2}^2 + {a_3}^2 + 2a_2a_3C_3 \\
        C_3 &= \frac{A^2 + B^2 - {a_2}^2 - {a_3}^2}{2a_2a_3} \\
        \theta_3 &= \cos^{-1}(\frac{A^2 + B^2 - {a_2}^2 - {a_3}^2}{2a_2a_3})
    \end{split}
\end{equation}
$$

The solutions of $\theta_3$ of the two different configurations are:
$$
\begin{equation}
    \theta_3 = \cos^{-1}(\frac{A^2 + B^2 - {a_2}^2 - {a_3}^2}{2a_2a_3})
      \text{ or } -\cos^{-1}(\frac{A^2 + B^2 - {a_2}^2 - {a_3}^2}{2a_2a_3})
\end{equation}
$$

#### Derivation on solving $\theta_2$

We subtlely change the expression of RHS as
$$
\begin{equation}
    \begin{cases}
        C_1(p_x - d_6a_x) + S_1(p_y - d_6a_y) - d_5S_{234} &= (a_2 + a_3C_3)C_2 - (a_3S_3)S_2 = A \\
        -p_z + d_1 + d_6a_z + d_5C_{234} &= (a_2 + a_3C_3)S_2 + (a_3S_3)C_2 = B
    \end{cases}
\end{equation}
$$

We add $AC_2$ to $BS_2$ and obtain
$$
\begin{equation}
    \begin{split}
        AC_2 + BS_2 &= a_2 + a_3C_3 \\
                    &= a_2 + a_3(\frac{A^2 + B^2 - {a_2}^2 - {a_3}^2}{2a_2a_3})
    \end{split}
\end{equation}
$$

Let $A = r\cos(\phi)$ and $B = r\sin(\phi)$, where $r = \sqrt{A^2 + B^2}$ and $\phi = \tan^{-1}{(\frac{B}{A})}$. The equation becomes:
$$
\begin{equation}
    \begin{split}
        r(C_{\phi}C_2 + S_{\phi}S_2) &= a_2 + a_3(\frac{A^2 + B^2 - {a_2}^2 - {a_3}^2}{2a_2a_3}) \\
        r\cos(\theta_2 - \phi) &= \frac{A^2 + B^2 + {a_2}^2 - {a_3}^2}{2a_2} \\
        \theta_2 - \phi &= \cos^{-1}(\frac{A^2 + B^2 + {a_2}^2 - {a_3}^2}{2a_2r}) \\
        \theta_2 &= \tan^{-1}{(\frac{B}{A})} + \cos^{-1}(\frac{A^2 + B^2 + {a_2}^2 - {a_3}^2}{2a_2r})
    \end{split}
\end{equation}
$$

The solutions of $\theta_2$ of the two different configurations are:
$$
\begin{equation}
    \begin{split}
        \theta_2 = \tan^{-1}{(\frac{B}{A})} - \cos^{-1}(\frac{A^2 + B^2 + {a_2}^2 - {a_3}^2}{2a_2r}) \\
        \text{ or } \tan^{-1}{(\frac{B}{A})} + \cos^{-1}(\frac{A^2 + B^2 + {a_2}^2 - {a_3}^2}{2a_2r})
    \end{split}
\end{equation}
$$

#### Derivation on solving $\theta_4$

Since we have obtained the value of $\theta_{234}$, we could easily calculate the value of $\theta_4$ as
$$
\begin{equation}
    \theta_4 = \theta_{234} - \theta_2 - \theta_3
\end{equation}
$$

#### Exception

The number of solution sets $\lbrace \theta_2, \theta_3, \theta_4 \rbrace$ would **degenerate from two to one if $S_3 \approx 0$.** Two cases are considered: $C_3 \approx 1$ and $C_3 \approx -1$.

##### [Case 1] $C_3 \approx 1$

The equation $(30)$ becomes
$$
\begin{equation}
    \begin{split}
        r^2 = A^2 + B^2 &= {a_2}^2 + {a_3}^2 + 2a_2a_3 \\
                    r^2 &= {(a_2 + a_3)}^2 \\
                    r   &= a_2 + a_3
    \end{split}
\end{equation}
$$

We denote $s = a_2 + a_3 - r$, and determine the occurrance of the exception by the value of $s$. If $s \approx 0$, then $C_3 \approx 1$.

The equation $(33)$ becomes
$$
\begin{equation}
    \begin{split}
        AC_2 + BS_2 &= a_2 + a_3 = r \\
        r\cos(\theta_2 - \phi) &= r \\
        \cos(\theta_2 - \phi) &= 1 \\
        \theta_2 - \phi &= 0 \\
        \theta_2 &= \tan^{-1}(\frac{B}{A})
    \end{split}
\end{equation}
$$

To sum up, the solution set $\lbrace \theta_2, \theta_3, \theta_4 \rbrace$ is listed as follows,
- $\theta_2 = \tan^{-1}(\frac{B}{A})$
- $\theta_3 \approx 0$
- $\theta_4 = \theta_{234} - \theta_2 - \theta_3$

##### [Case 2] $C_3 \approx -1$

The equation $(30)$ becomes
$$
\begin{equation}
    \begin{split}
        r^2 = A^2 + B^2 &= {a_2}^2 + {a_3}^2 - 2a_2a_3 \\
                    r^2 &= {(a_2 - a_3)}^2 \\
                    r   &= |a_2 - a_3|
    \end{split}
\end{equation}
$$

We denote $t = r - |a_2 - a_3|$, and determine the occurrance of the exception by the value of $t$. If $t \approx 0$, then $C_3 \approx -1$.

The equation $(33)$ becomes
$$
\begin{equation}
    \begin{split}
        AC_2 + BS_2 &= a_2 - a_3 = r \\
        r\cos(\theta_2 - \phi) &= r \\
        \cos(\theta_2 - \phi) &= 1 \\
        \theta_2 - \phi &= 0 \\
        \theta_2 &= \tan^{-1}(\frac{B}{A})
    \end{split}
\end{equation}
$$

To sum up, the solution set $\lbrace \theta_2, \theta_3, \theta_4 \rbrace$ is listed as follows,
- $\theta_2 = \tan^{-1}(\frac{B}{A})$
- $\theta_3 \approx \pi$
- $\theta_4 = \theta_{234} - \theta_2 - \theta_3$