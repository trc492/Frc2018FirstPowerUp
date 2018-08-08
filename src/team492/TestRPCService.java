package team492;

import com.github.arteam.simplejsonrpc.client.JsonRpcId;
import com.github.arteam.simplejsonrpc.core.annotation.JsonRpcMethod;
import com.github.arteam.simplejsonrpc.core.annotation.JsonRpcService;

@JsonRpcService
public interface TestRPCService 
{
	@JsonRpcMethod
	public TestRPCClass getMajiraInstance();
}
