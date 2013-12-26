import com.illposed.osc.*;

class Receiver {

	public static void main(String[] args) throws java.net.SocketException {
		int receiverPort = 8000;
		String adress = "/test" ;
		
		if ( args.length > 0 ){
			receiverPort = Integer.parseInt(args[0]) ;
		}
		if ( args.length > 1 ){
			adress = args[1] ;
		}
		
		OSCPortIn receiver = new OSCPortIn(receiverPort);

		OSCListener handler = new OSCListener() {
			public void acceptMessage(java.util.Date time, OSCMessage message) {
				System.out.println("Handler1 called with address "
						+ message.getAddress());
				for ( Object i : message.getArguments() ) {
					System.out.print(i + " ") ;
				}
				System.out.println("") ;
			}
		};

		receiver.addListener(adress, handler);

		System.out.println("Server is listening on port " + receiverPort
				+ "...");
		receiver.startListening();
	}

}
