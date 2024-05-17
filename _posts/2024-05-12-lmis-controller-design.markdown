---
layout: post
title:  "Linear Matrix Inequalities for designing a robust controller"
date:   2024-05-12
categories: control-theory
excerpt_separator: <!--more-->
published: true
---

Linear Matrix Inequalities (LMIs) are somewhat of a swiss army knife in control theory. Some examples of application of LMIs include: robust controller synthesis, optimal controller synthesis, stability of linear (hybrid) systems, fault detecting and even constraints in MPC problems [1, 2]. While a very powerful and versatile tool, LMIs can seem a bit daunting and theoretical. This post will attempt to practically introduce the topic and show through an example in `julia` how LMIs can be used to synthesize an robust controller.

<!--more-->

### Defining LMIs
Before we can do anything, we need to actually define what an LMI is. Let us consider some symmetric real matrix $P \in \mathbb{R}^{n\times n}$. In linear algebra there exists a concept of the "definite-ness" of a matrix. The exact definition for this can be given as

$$
\text{$P$ is positive definite } \iff x^{T}Px > 0 \quad \forall x \in \mathbb{R}^{n}
$$

For semi-definite, the strict inequality becomes a regular inequality and for negative (semi-) definite matrices, the direction of the inequality flips. This property allows us to induce a partial ordering on matrices. That is, we can define some notion of matrix $P$ beging "bigger than" matrix $Q$. This is done by defining:

$$
P \succ Q \iff P - Q \succ 0
$$

Note however that, again, this is a _partial_ ordering. This means that $P\nsucc Q$ does NOT imply that $P \preceq Q$! The result of the subtraction $P-Q$ can also be non-definite. We can use this property to than define inequalities of the form

$$
F(x) = F_{0} + x_{1}F_{1} + \cdots x_{N}F_{N} \succeq 0
$$

Which is a _linear matrix inequality_ in the variable $x$! An especially interesting property is that $x$ can be scalar, vector or even a matrix.

### A bit of Lyapunov Theory
So now that we know what an LMI actually is we need to see how we can apply this to a control problem. To do this we need to briefly talk about Lyapunov fucntions.
Consider some (nonlinear) dynamical system

$$
\dot x = f(x) \quad x(t) \in \mathbb{R}^{n}
$$

Some equilibrium point of this system is Lyapunov stable there exists some Lyapunov function $V(x)$ which has the following properties:

1. $V(x) > 0,\ \forall x(t) \in \mathbb{R}^{n}$
2. $V(0) = 0$
3. $\dot V(x) = \nabla V \cdot f(x) < 0, \ \forall  x(t) \in \mathbb{R}^{n}$

A function satisfying 1. and 2. is sometimes called a Lyapynov candidate. _If_ (which in the nonlinear case is a big if) we can find a function $V(x)$ which satisfies all 3 properties it follows that the equilibrium point is Lyapunov stable. Luckily, in the linear case we can quite easily find a Lyapunov candidate by taking $V(x) = x^{T}Px$ with $P \succ 0$. Now for this to actually be a Lyapunov function, the Lyapunov inequality 

$$
A^{T}P + PA + Q \preceq 0
$$

must also hold. Thus we have a set of 2 LMIs on $P$ which, if satisfied show that there exists a Lyapunov function, proving that the system is stable.


### A Parametrically Uncertain System?!
In order to show an application of the Lyapuynov inequality we will consider a system $(A, B)$ which has some _parametric uncertainty_ in the system dynamics. That is, we know our system $A$ which we might have obtained through system identification techniques however we are not very confident in the entries $a_{11}$ and $a_{22}$. We estimate that these entries may be slightly bigger or slightly smaller than what we identified. We thus express an entire range of system matrices $A$ that our system _could_ be:

$$
\begin{gather*}
A(\theta_{1}, \theta_{2}) = \begin{bmatrix}
    1 + \theta_{1} & 2\\
    0 & 1 + \theta_{2}\\
\end{bmatrix} \quad \theta_{1}, \theta_{2} \in [-0.5, 0.5]\\

B = \begin{bmatrix}
    0\\
    1
\end{bmatrix}
\end{gather*}
$$

Let's now compare the following 2 approaches for finding a controller:

1. LQR control for the nominal system ($\theta_{1}, \theta_{2}) = (0, 0)$
2. Controller  based on LMIs to guarantee robustness


### Using LQR on the nominal system

We (poorly) choose the LQR gains to be $Q = \text{diag}(1, 1)$, $R=1$ for the LQR approach. 

```julia
using ControlSystems

A = [1 2; 0 1]
B = [0; 1]
Q = diag([1, 1])
R = 1

K = lqr(A, B, Q, R)
sys_lqr = ss(A-B*K, B, [1 0], 0)
```

### Using LMIs to find a controller

Now let's do the LMI case. We wish that the closed loop system $A+BK$ (using positive sign convention for ease of notation) is Lyapunov stable for all possible values of $A$. Let's first see if we can find a condition to find the controller $K$ using LMIs. We of course want the closed loop system to be Lyapunov stable which results in the following Lyapunov inequality

$$
(A + BK)^{T}P + P(A + BK) \prec 0
$$

This can be rewritten as

$$
A^{T}P + B^{T}K^{T}P + PA + PBK \prec 0
$$

Now this matrix inequality is actually **non-linear** with respect to the parameters $P$ and $K$. To deal with this we introduce something called a congurence transformation. To keep it (very) brief: A congruence transformation of $P$ is the transformation $T^{T}PT$ for some non-singular $T$. The useful property of this transformation is that it preservers the definite-ness of $P$. That is: $P \succ 0 \implies T^{T}PT \succ 0$. If we now let $T=P^{-1}$ and using the fact that $P$ (and by extention $P^{-1}$) are symmetric it follows that

$$
P^{-1}A^{T} + P^{-1}K^{T}B^{T} + AP^{-1} + BKP^{-1} \prec 0
$$

With the change of variables $Y = KP^{-1}$ and $X = P^{-1}$ we end up with an LMI again. 

$$
XA^{T} +Y^{T}B^{T} + AX + BY \prec 0
$$

Now technically we need to check all possible pairs $(\theta_{1}, \theta_{2})$ with the LMI above resulting in an infinite amount of LMIs. We can however greatly reduce this to just 4 LMIs by evaluating all the "corner points" of the set of systems:

$$
\begin{align*}
A_{1} &= A(0.5, 0.5)\\
A_{2} &= A(0.5, -0.5)\\
A_{3} &= A(-0.5, 0.5)\\
A_{4} &= A(-0.5, -0.5)\\
\end{align*}
$$

Resulting in just 5 LMIs to find a Stabilizing controller:

$$
\begin{align*}
X &\succ 0\\
XA_{1}^{T} +Y^{T}B^{T} + A_{1}X + BY &\prec 0\\
XA_{2}^{T} +Y^{T}B^{T} + A_{2}X + BY &\prec 0\\
XA_{3}^{T} +Y^{T}B^{T} + A_{3}X + BY &\prec 0\\
XA_{4}^{T} +Y^{T}B^{T} + A_{4}X + BY &\prec 0\\
\end{align*}
$$

Now let's implement this in Julia usign `Convex.jl` and `SCS.jl`. For technical reasons, we will add a positive definite matrix $S$ the left-hand side of all the LMIs on $X$ and $Y$ making them non-strict. Import the neccesary libraries

```julia
using Convex
using SCS
using ControlSystems
```

Define the systems:

```julia
A = (θ1, θ2) -> [1+θ1 2; 0 1+θ2];

A1 = A(-0.5, -0.5)
A2 = A(0.5, 0.5)
A3 = A(-0.5, 0.5)
A4 = A(0.5, -0.5)
```

and set up and solve the convex optimisation problem:

```julia
S = diag([1 1])

# Set up the SDP problem
X = semidefinite(2)
Y = Variable(1, 2)

constraints = [
    X ⪰ 0,
    X*A1' + Y'*B' + A1*X + B*Y + S ⪯ 0,
    X*A2' + Y'*B' + A2*X + B*Y + S ⪯ 0,
    X*A3' + Y'*B' + A3*X + B*Y + S ⪯ 0,
    X*A4' + Y'*B' + A4*X + B*Y + S ⪯ 0,
]
problem = satisfy(constraints)
solve!(problem, SCS.Optimizer())

K = Y.value*inv(X.value)
sys_robust = ss(A-B*K, B, [1 0], 0)
```


### Comparing results
Now the moment of truth: comparing the LQR result with the LMI result! For all systems regulation towards $0$ with initial conditions $[0; 1]$ will be considered. Something immediately of note is that the LQR controller actually becomes **unstable** for some values of $(\theta_1, \theta_{2})$. We could not have known this ahead of time eihter, the robustness of the LQR controller had to be checked _a posteriori_. For the LMI controller this was actually not the case as we specifically constrained it to stabilize all possible systems in the set, which is why LMIs are particularly useful for problems like this. The robust controller shows some slight variations in performance for different values of $\theta_{1}$ and $\theta_{2}$, however all of them are definitely stable which is exactly what we wanted.

{:refdef: style="text-align: center;"}
![](/assets/images/lmis-01-comparison-1.png)
{: refdef}




### Concluding Remarks
This post showed a fun little experiment with LMIs and showing one of it's many uses in control theory. In theory, we could combine both the LQR and LMI approach by expressing the LQR problem in terms of LMI contraints. This will then result in an LQR optimal controller that is guaranteed to stabilize all the systems within the set! There are many different direction we could take from here using LMIs but those will be left as future experiments. 

I hope this post piqued your curiosity about the many applications of LMIs as well as helped illustrate how we would practically implement these when designing controllers.




### Citations

[1] Scherer CW, Weiland S. Linear matrix inequalities in control. In Levine WS, editor, The Control Systems Handbook, Second Edition: Control System Advanced Methods. London: CRC Press. 2011. p. 24/1-24/30

[2] E. Granado, W. Colmenares, J. Bernussou, and G. Garcia, “LMI BASED MPC,” IFAC Proceedings Volumes, vol. 35, no. 1, pp. 177–182, 2002, doi: https://doi.org/10.3182/20020721-6-ES-1901.00598. 
