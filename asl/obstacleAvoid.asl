safety(bumper).
@Bumper [atomic]
+bumper(pressed)
	<-	.broadcast(tell, avoid(obstacle));
		!drive(backward);
		!turn(left);
		!drive(forward);
		!turn(right);
		!drive(forward);
		!drive(forward);
		!turn(right);
		!drive(forward);
		!turn(left).
		


