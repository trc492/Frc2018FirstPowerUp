package team492;

import java.io.BufferedWriter;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.net.Socket;

public class AwooCommunicator
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
}

