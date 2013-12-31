import com.illposed.osc.*;

class Receiver {

	public static void main(String[] args) throws java.net.SocketException {
		int receiverPort = 8000;
		String[] adress = new String[]{"/test"} ;
		
		if ( args.length > 0 ){
			receiverPort = Integer.parseInt(args[0]) ;
		}
		if ( args.length > 1 ){
			adress = new String[args.length -1] ;
			for ( int i=1 ; i<args.length ; i++ ){
				adress[i-1] = args[i] ;
			}
		}
		
		OSCPortIn receiver = new OSCPortIn(receiverPort);

		OSCListener handler = new OSCListener() {
			public void acceptMessage(java.util.Date time, OSCMessage message) {
				System.out.println("[OSC Receiver] Handler1 called with address "
						+ message.getAddress());
				for ( Object i : message.getArguments() ) {
					System.out.print(i + " ") ;
				}
				System.out.println("") ;
			}
		};

		for ( int i=0 ; i<adress.length ; i++ ){
			receiver.addListener(adress[i], handler);
		}

		System.out.println("Server is listening on port " + receiverPort
				+ "...");
		receiver.startListening();
	}

}
