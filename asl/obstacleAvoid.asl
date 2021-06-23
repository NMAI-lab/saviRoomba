safety(obstacle).
@obstacleAvoidance [atomic]
+obstacle(Distance)
	:	Distance < 5
	<-	//!controlSpeed(0.0);
		steering(-0.3);
		.broadcast(tell, obstacleAvoid(steering(-0.3))).
		
safety(proximity).
@obstacleAvoidanceProximity [atomic]
+proximity(_)   <-  drive(stop).

safety(impact).
@obstacleAvoidanceImpact [atomic]
+impact(_)      <-  drive(stop).
