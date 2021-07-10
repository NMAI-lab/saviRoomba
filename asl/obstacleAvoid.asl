safety(bumper).
@Bumper [atomic]
+bumper(pressed)
	<-	.broadcast(tell, avoid(obstacle));
		!turn(left);
		!move(0.5,-1,5).
		


