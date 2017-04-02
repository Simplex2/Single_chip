package com.bluelight.demo.service;

import java.util.Map;

import org.apache.commons.codec.binary.Base64;

import com.bluelight.demo.api.DeviceApi;
import com.bluelight.demo.api.MpApi;
import com.bluelight.demo.consts.MsgType;
import com.bluelight.demo.consts.XmlResp;
import com.bluelight.demo.mock.DBMock;
import com.bluelight.demo.protocol.BlueLight;
import com.bluelight.demo.protocol.BlueLight.CmdId;

/**
 * 回调业务处理
 */
public class CallbackService {

	// 自定义菜单中的key值
	public static final String V1001_LIGHT_ON = "V1001_LIGHT_ON";//点灯
	public static final String V1002_LIGHT_OFF = "V1002_LIGHT_OFF";//灭灯
	
	public String handle(Map<String, String> reqMap) throws Exception {
		String msgType = reqMap.get("MsgType");
		String fromUser = reqMap.get("FromUserName");
		String toUser = reqMap.get("ToUserName");

		// 针对不同类型的消息和事件进行处理

		// 文本消息
		if (MsgType.TEXT.equals(msgType)) {
			// 可以在此处进行关键字自动回复
			String content = "收到文本消息：" + reqMap.get("Content");
			return XmlResp.buildText(fromUser, toUser, content);
		}
		
		// 基础事件推送
		if (MsgType.EVENT.equals(msgType)) {
			String event = reqMap.get("Event");
			// 关注公众号
			if (MsgType.Event.SUBSCRIBE.equals(event)) {
				// 回复欢迎语
				return XmlResp.buildText(fromUser, toUser, "欢迎关注蓝牙灯泡demo测试公众号！");
			}
			// 菜单点击事件
			if (MsgType.Event.CLICK.equals(event)) {
				// 根据key值判断点击的哪个菜单
				String eventKey = reqMap.get("EventKey");
				// 点灯/灭灯
				if (V1001_LIGHT_ON.equals(eventKey) 
						|| V1002_LIGHT_OFF.equals(eventKey)) {
					//是否点灯操作
					boolean open = V1001_LIGHT_ON.equals(eventKey);

					// 根据 fromUserName 获取绑定的信息
					Map<String, String> boundInfo = DBMock.queryBoundInfo(fromUser);

					// 未绑定
					if (boundInfo == null) {
						return XmlResp.buildText(fromUser, toUser, "未绑定");
					}

					String deviceType = boundInfo.get("deviceType");
					String deviceID = boundInfo.get("deviceID");
					String openID = boundInfo.get("openID");
					
					// 构造设备消息
					CmdId cmdId = open ? BlueLight.CmdId.OPEN_LIGHT_PUSH : BlueLight.CmdId.CLOSE_LIGHT_PUSH;
					byte[] respRaw = BlueLight.build(cmdId, null, (short)0).toBytes();
					// Base64编码
					final String content = Base64.encodeBase64String(respRaw);
					
					// 推送消息给设备
					DeviceApi.transMsg(deviceType, deviceID, openID, content);
					
					// 回复
					boolean debug = true;
					if(debug){
						// 返回调试信息
						String debugText = "已发送" + (open ? "点灯" : "灭灯") + "消息：" + "deviceID为" + deviceID + ",设备消息为" + content;
						return XmlResp.buildText(fromUser, toUser, debugText);
					}else{
						return "";
					}
				}
			}
		}

		// 设备消息或事件
		if (MsgType.DEVICE_EVENT.equals(msgType)
				|| MsgType.DEVICE_TEXT.equals(msgType)) {
			String reqContent = reqMap.get("Content");
			String deviceType = reqMap.get("DeviceType");
			String deviceID = reqMap.get("DeviceID");
			String sessionID = reqMap.get("SessionID");
			final String openID = reqMap.get("OpenID");
			// 设备事件推送
			if (MsgType.DEVICE_EVENT.equals(msgType)) {
				String event = reqMap.get("Event");
				// 绑定/解绑事件
				if (MsgType.DeviceEvent.BIND.equals(event)
						|| MsgType.DeviceEvent.UNBIND.equals(event)) {
					// 存储用户和设备的绑定关系
					if(MsgType.DeviceEvent.BIND.equals(event)){
						DBMock.saveBoundInfo(reqMap);
					}else{
						DBMock.removeBoundInfo(reqMap.get("FromUserName"));
					}
					// 设备绑定/解绑事件可以回复空包体
					return "";
				}
			}
			// 收到设备消息
			if (MsgType.DEVICE_TEXT.equals(msgType)) {
				// Base64解码
				byte[] reqRaw = Base64.decodeBase64(reqContent);
				// 反序列化
				BlueLight lightReq = BlueLight.parse(reqRaw);
				
				// 逻辑处理
				// demo中 推送消息给用户微信
				String reqText = lightReq.body;
				System.out.println("recv text:" + reqText);
				String transText = "收到设备发送的数据：";
				
				byte[] reqTextRaw = reqText.getBytes("UTF-8");
				if (reqTextRaw.length > 0 && reqTextRaw[reqTextRaw.length - 1] == 0) {
					// 推送给微信用户的内容去掉末尾的反斜杠零'\0'
					transText = transText + new String(reqTextRaw, 0, reqTextRaw.length - 1, "UTF-8");
				} else{
					transText = transText + reqText;
				}
				
				// 推送文本消息给微信
				MpApi.customSendText(openID, transText);

				// demo中 回复 收到的内容给设备
				BlueLight lightResp = BlueLight.build(BlueLight.CmdId.SEND_TEXT_RESP, reqText, lightReq.head.seq); 
				// 序列化
				byte[] respRaw = lightResp.toBytes();
				// Base64编码
				String respCon = Base64.encodeBase64String(respRaw);
				
				// 设备消息接口必须回复符合协议的xml
				return XmlResp.buildDeviceText(toUser, fromUser, deviceType, deviceID, respCon, sessionID);
			}
		}

		// 未处理的情况返回空字符串
		return "";
	}
}
