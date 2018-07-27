package team492;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;
import java.net.Socket;

import com.github.arteam.simplejsonrpc.client.Transport;

public class AwooCommunicator implements Transport
{
    private int port;
    private BufferedWriter bw;
    private Socket kemono;
    
    public AwooCommunicator(int port)
    {
        this.port = port;
    }
    
    public void initCommunicator()
    {
        try
        {
        	System.out.println("Attempting communication to port " + port);
            kemono = new Socket("127.0.0.1", port);
            bw = new BufferedWriter(new OutputStreamWriter(kemono.getOutputStream()));
        }
        catch (Exception e)
        {
            e.printStackTrace();
        }
    }
    
    public void sendMessage(String msg)
    {
        try
        {
            if (kemono == null)
            {
                return;
            }
            if(!kemono.isClosed() && kemono.isConnected())
            {
                try
                {
                    bw.write(msg+"\n");
                    bw.flush();
                }
                catch (IOException ioe)
                {
                    ioe.printStackTrace();
                }
            }
        }
        catch (Exception e)
        {
            e.printStackTrace();
        }
    }
    
    public void implodeCommunicator()
    {
        try
        {
            bw.close();
        }
        catch (IOException e)
        {
            e.printStackTrace();
        }
        bw = null;
        try
        {
            kemono.close();
        }
        catch (IOException e)
        {
            e.printStackTrace();
        }
        kemono = null;
        port = -1;
    }

	@Override
	public String pass(String request) throws IOException
	{
		sendMessage(request);
		String toRet = "";
		InputStream istream = kemono.getInputStream();
		BufferedReader receiveRead = new BufferedReader(new InputStreamReader(istream));
		String receiveMessage;
		receiveMessage = receiveRead.readLine();
		while(receiveMessage != null)
		{
			toRet += (receiveMessage + "\n");
		}
		return toRet;
	}
}

