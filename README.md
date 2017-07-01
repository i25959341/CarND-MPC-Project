# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

## The Model

The vehicle model used in this project is a kinematic bicycle model. It neglects all dynamical effects such as inertia, friction and torque. The model uses the following six variables

* _x, y_ position
* velocity _v_
* heading _psi_
* turn rate _epsi_
* cross-track error _cte_
	
The model consists of the following equations:

```
      // x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
      // y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
      // psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
      // v_[t+1] = v[t] + a[t] * dt
      // cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
      // epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
```

We model two actuators to drive the car around the track:

* Steering adjustment (-1 to 1), _delta_, varying +/- 25 degrees
* Throttle adjustment (-1 to 1), _a_, varying from full power reverse to full power forward

### Cost Function Parameters
The cost of a trajectory of length N is computed as follows
```
	cost = sum_over_time(
		            cte(i)^2    //  we want cte=0, espi=0, v=v_ref=30
              + epsi(i)^2   //  we want espi=0
              + (v(i)-v_ref)^2  //v=v_ref
              + delta(i)^2   // Minimise use of actuation
              + 10 a(i)^2   // Minimise use of actuation
              + 600 [delta(i+1)-delta(i)]  // smooth actuation
              + [a(i+1)-a(i)]  // // smooth actuation
	) 
 ```
