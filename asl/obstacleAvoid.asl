safety(bumper).
@Bumper [atomic]
+bumper(pressed)
	<-	.broadcast(tell, avoid(obstacle));
		!turn(left);
		!move(0.5,-3,10);
		!turn(left)
		!turn(left)
		!turn(left).
		


