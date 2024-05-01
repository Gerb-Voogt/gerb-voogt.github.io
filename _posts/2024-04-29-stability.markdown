---
layout: post
title:  "Stability Criteria for Linear Systems in Continuous and Dicrete Time"
date:   2024-04-29
categories: [control-theory, mathematics]
excerpt_separator: <!--more-->
---
This post is a rough draft and only here as a placeholder! 
Discrete time systems are an important part of modern control theory due to the wide application of computer control. An interesting aspect of the difference between continuous- and discrete-time systems is the difference in stability criteria for in particular linear system. In this article we will explore these differences and try to explain why the criteria arise very naturally if consider the properties of the systems.

<!--more-->

Generally system models are derived through first principles and differential equations. This results in (likely non-linear) models of the form 
\begin{gather}
\dot x = f(t, x, u)
\end{gather}
Where $x \in \mathbb{R}^{n}$ is the state of the system, $u \in \mathbb{R}^{m}$ is the control input and $f: \mathbb{R}^{n} \to \mathbb{R}^{n}$ is a vector valued function. This is great in theory, however in practice when concering ourselves with control of some system we wish to use a computer. Computers are fundamentally digital devices being driven by a clock and thus cannot operate on continuous signals directly. In order to analyse stability and performance of these system we must apply discretization in order to yield a discrete representation of this signal. The discrete version of the system can be given as
\begin{gather}
x_{k+1} = f(t_{k}, x_{k}, u_{k})
\end{gather}


# The linear systems case
In the case that a system just so happens to be linear (or can be linearised around some equilibrium point or trajectory) the continuous dynamics can instead be written in the familiar linear state-space form
\begin{gather}
\dot x = Ax + Bu
\end{gather}
which after sampling becomes 
\begin{gather}
x_{k+1} = \Phi(h) x_{k} + \Gamma(h) u_{k}
\end{gather}
Something of note here is the changing of stability criteria between continuous and discrete time. Generally for continuous systems we say that the neccesary condition for stability is that the real part of all eigenvalues needs to be smaller than or equal to $1$ for all possible eigenvalues of the system matrix $A$. That is
\begin{gather}
\text{Re}(\lambda_{i}(A)) < 0,\; \forall i \in \{0, 1, \cdots , n\}
\end{gather}
Instead for discrete systems (assuming constant sampling time) we say instead that all eigenvalues need to be strictly bounded by $1$ in magnitude:
\begin{gather}
|\lambda_{i}(\Phi)| < 1,\; \forall i \in \{0, 1, \cdots , n\}
\end{gather}


# Explaining the difference in stability criteria
So, what gives? Why these differences in stability criteria? The easiest way to reason about this is to pick a simple problem and analyse the behaviour of these systems. We shall then generalize these to $n$ dimensional systems and show where the differences arise from.

In the 1D case, the system $\dot x = ax$ is a simple linear first-order differential which can easily be solved by integrating resulting in
\begin{gather}
x = e^{a(t-t_{0})}x_{0}
\end{gather}
It can easily be verified that 
\begin{gather}
\lim_{t\to \infty} e^{a(t-t_{0})} = 0\; \text{ if } a < 0
\end{gather}
\begin{gather}
\lim_{t\to \infty} e^{a(t-t_{0})} \to \infty\; \text{ if } a > 0
\end{gather}
We now want to see how we can relate this one dimensional system to the $n$-dimensional case. To do so let's consider the linear system of differential equations
\begin{gather}
\dot x = Ax
\end{gather}
Using a spectral decomposition it can be shown that the solution can be written in terms of the eigenvalues $\lambda_{i}$ and eigenvectors $v_{i} \in \mathbb{R}^{n}$ of the system as 
\begin{gather}
x(t) = \sum_{i=1}^{n} v_{i}e^{\lambda_{i}(t-t_{0})}x_{0}
\end{gather}
From this the same reasoning as for the simplified case applies! If $\lambda_{i} > 0$ for any $i$, than atleast $1$ component of $x(t)$ will explode to $\infty$ as $t$ becomes large, making the system unstable!


Let's now instead consider the discrete-time version of the system above
\begin{gather}
x_{k+1} = \Phi x_{k}
\end{gather}
This system is, as opposed to the CT system, a finite difference equation rather than a differential equation. That is, the next vector $x_{k+1}$ is some linear transformation of the previous vector. An approach would be to write the initial vector $x_{0}$ as a linear combination of the eigenvectors of $\Phi$ with normalised weights $c_{i} \in [0, 1]$
\begin{gather}
x_{0} = c_{1}v_{1} + c_{2}v_{2} + \cdots + v_{n}c_{n}
\end{gather}
A single application of $\Phi$ scales all of these eigenvectors by their eigenvalue $\lambda_{i}$. Let's assume that for ease of argumentation that the eigenvalues are ordered such that $\lambda_{1} < \lambda_{2} < \cdots < \lambda_{n}$. This than yields
\begin{gather}
x_{1} = c_{1}\lambda_{1}v_{1} + c_{2}\lambda_{2}v_{2} + \cdots + c_{n}\lambda_{n}v_{n}
\end{gather}
Which after $N$ applications becomes
\begin{gather}
x_{N} = c_{1}\lambda_{1}^{N}v_{1} + c_{2}\lambda_{2}^{N}v_{2} + \cdots + c_{n}\lambda_{n}^{N}v_{n}
\end{gather}
It can be argued that this sum is dominated by the largest eigenvalue of $\Phi$ as $N$ becomes sufficiently large. It hence follows that 
\begin{gather}
x_{N} \approx c_{n}\lambda_{n}^{N}v_{n}
\end{gather}
In the limiting case, it follows that 
\begin{gather}
\lim_{N\to \infty} x_{N} = \lim_{N\to \infty} c_{n}\lambda_{n}^{N}v_{n}
\end{gather}
Which has 2 cases (ignoring the $\lambda = 1$ case for now)
\begin{gather}
\lim_{N\to \infty} c_{n}\lambda_{n}^{N}v_{n} = \infty \quad \text{ if } \lambda_{n} > 1\newline
\lim_{N\to \infty} c_{n}\lambda_{n}^{N}v_{n} = 0 \quad \text{ if } \lambda_{n} < 1
\end{gather}
Thus it clearly follows that the largest eigenvalue must be bounded by $1$ to remain stable in the discrete case!
