---
layout: post
title:  "Linear Matrix Inequalities for designing a controller"
date:   2024-05-12
categories: control-theory
excerpt_separator: <!--more-->
published: true
---

Linear Matrix Inequalities (LMIs) are somewhat of a swiss army knife in control theory. Some examples of application of LMIs include: robust controller synthesis, optimal controller synthesis, stability of linear (hybrid) systems, fault detecting and even constraints in MPC problems [1, 2]. While a very powerful and versatile tool, LMIs can seem a bit daunting and theoretical. This post will attempt to gently introduce the topic and show through an example in `julia` how LMIs can be used to synthesize an LQR controller.

<!--more-->

### Defining LMIs
Let us consider some symmetric real matrix $P \in \mathbb{R}^{n\times n}$. In linear algebra there exists a concept of the "definite-ness" of a matrix. The exact definition for this can be given as

$$
\text{$P$ is positive definite } \iff x^{T}Px > 0 \quad \forall x \in \mathbb{R}^{n}
$$

For semi-definite, the strict inequality becomes a regular inequality and for negative (semi-) definite matrices, the direction of the inequality flips.
Loosely speaking, an intuition could be that all the eigenvalue of a positive definite matrix are positive (or negative in the negative definite case) which causes $P$ to stretch the space $\mathbb{R}^{n}$ in the direction of it's eigenvectors. [ILLUSTRATE THIS WITH A PLOT]

An interesting consequence of this property is that it allows inducing a partial ordering on matrices. 

$$
P \succ Q \iff P - Q \succ 0
$$

Note however that, again, this is a _partial_ ordering. This means that $P\nsucc Q$ does NOT imply that $P \preceq Q$! Matrices can also just simply be not definite.

### A bit of Lyapunov Theory
In order to design any stabalizing controller it is very useful to know how we can actually define stability of a system. A common way in control engineering to do this is by analysing the eigenvalues of the system. For continuous system stability is guaranteed through all poles being strictly open left-half plane, and for discrete time instead through the condition that all poles are strictly inside the complex unit disk. These stability properties are sometimes also referred to as the system being Hurwitz (continuous time) or Schur (Discrete time). Thinking of the system in terms of it's eigenvalues generally works quite well, as they are quiet easy to compute and intuitive to analyse. We however quickly run into problems when considering for example switching systems or uncertain systems. For example what are the eigenvalues of the following system?

$$
A(\theta) = \begin{bmatrix}
    a+\theta_{1} & b\\
    c & d+\theta_{2}
\end{bmatrix} \quad \theta_{1}, \theta_{2} \in [0, 1]
$$

In this case, there exists an infinite set of possible eigenvalues of $A$. LMIs provide us with an excellent framework for analysing stability of such systems through the use of LMIs and Lyapunov theory!

Consider some (nonlinear) dynamical system

$$
\dot x = f(x) \quad x(t) \in \mathbb{R}^{n}
$$


Some equilibrium point of this system is Lyapunov stable by showing the existence of some Lyapunov function $V(x)$ which has the following properties:

1. $V(x) > 0,\ \forall x(t) \in \mathbb{R}^{n}$
2. $V(0) = 0$
3. $\dot V(x) = \nabla V \cdot f(x) < 0, \ \forall  x(t) \in \mathbb{R}^{n}$

A function satisfying 1. and 2. is sometimes called a Lyapynov candidate. _If_ we can find this function $V(x)$ (and thus show it's existence) it follows that the equilibrium point is Lyapunov stable. The Lyapunov function can be interpreted as generalized energy of the system. This theorem essentially says the following

> If the energy of the system strictly decreases when approaching the equilbrium point, and the energy is minimized in the equilibrium point itself, then it follows that the state asymptotically goes to the equilibrium point.

So this is great, we just need to find a Lyapunov function to show stability. Simple right? Well no. There exists no algorithm for finding a Lyapunov function. In case th system is mechanical, energy could be used but this will not work in general. Luckily, in the case of a linear system, we know that a quadratic Lyapunov function will always be a valid candidate! Thus if our system can be given as $\dot x = Ax$, then Lyapunov function would be:

$$
V(x) = x^{T}Px, \; P \succ 0
$$

And look, an LMI! A somewhat trivial LMI but an LMI nonetheless. Now let's see what the time derivative of the Lyapunov function looks like:

$$
\begin{align*}
\dot V(x) &= \dot x^{T}Px + x^{T}P\dot x\\
&= x^{T}A^{T}Px + x^{T}PAx\\
&= x^{T}(A^{T}P + PA)x\\
\end{align*}
$$

Now for this to be a valid Lyapunov function we must have $\dot V(x) \leq 0$, which leads to a second LMI:

$$
A^{T}P + PA \prec 0 \iff 
A^{T}P + PA  + Q \preceq 0 \quad (Q \succ 0)
$$

Thus we can establish that a system $A$ is Lyapunov stable if there exists some matrix $P$ which satsifies the constraints

$$
\begin{gather*}
P \succ 0\\
A^{T}P + PA  + Q \preceq 0
\end{gather*}
$$



### Citations

[1] Scherer CW, Weiland S. Linear matrix inequalities in control. In Levine WS, editor, The Control Systems Handbook, Second Edition: Control System Advanced Methods. London: CRC Press. 2011. p. 24/1-24/30

[2] E. Granado, W. Colmenares, J. Bernussou, and G. Garcia, “LMI BASED MPC,” IFAC Proceedings Volumes, vol. 35, no. 1, pp. 177–182, 2002, doi: https://doi.org/10.3182/20020721-6-ES-1901.00598. 
