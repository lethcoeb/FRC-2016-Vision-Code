package TT;

import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.Inet4Address;
import java.net.InetAddress;

public class ServerThread extends Thread {
	
	DatagramSocket socket;
	String msg;
	byte[] msgByte;
	InetAddress host;
	int serverSocket;
	DatagramPacket request;
	
	public ServerThread(){
		init();
	}
	
	public void init(){
		
		try{
			socket = new DatagramSocket();
			host = InetAddress.getByName("localhost");
			//can be between 5800 and 5810
			serverSocket = 5803;
		}catch(Exception e){
			System.out.println("Error: + " + e);
		}
		
	}
	
	public void sendInfo(boolean targetFound, double angle, double distance){
		
		msg = String.valueOf(targetFound) + ";" + String.valueOf(angle) + ";" + String.valueOf(distance);
		msgByte = msg.getBytes();
		
		try{
			request = new DatagramPacket(msgByte, msgByte.length, host, serverSocket);
			socket.send(request);
		}catch(Exception e){
			System.out.println("Error: " + e);
		}
		
	}
	
}
