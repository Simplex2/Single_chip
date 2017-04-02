package com.bluelight.demo.api;

import net.sf.json.JSONObject;

import com.bluelight.demo.api.json.AccessToken;
import com.bluelight.demo.api.util.HttpUtil;
import com.bluelight.demo.consts.WxConfig;

/**
 * 公众平台API 设备专用API定义在DeviceApi
 */
public class MpApi {
	
	private static final String GetAccessTokenUrl = "https://api.weixin.qq.com/cgi-bin/token?grant_type=client_credential&appid="
			+ WxConfig.APPID + "&secret=" + WxConfig.APPSECRET;
	private static final String CustomSendUrl = "https://api.weixin.qq.com/cgi-bin/message/custom/send?access_token=ACCESS_TOKEN";
	private static final String CreateMenuUrl = "https://api.weixin.qq.com/cgi-bin/menu/create?access_token=ACCESS_TOKEN";
	private static final String QueryMenuUrl = "https://api.weixin.qq.com/cgi-bin/menu/get?access_token=ACCESS_TOKEN";
	private static final String DeleteMenuUrl = "https://api.weixin.qq.com/cgi-bin/menu/delete?access_token=ACCESS_TOKEN";

	/**
	 * 获取访问凭证
	 * <p>
	 * 正常情况下access_token有效期为7200秒，重复获取将导致上次获取的access_token失效。
	 * 由于获取access_token的api调用次数非常有限，需要全局存储与更新access_token
	 * <br/>
	 * 文档位置：基础支持->获取access token
	 */
	public static AccessToken getAccessToken() {
		String resultContent = HttpUtil.executeGet(GetAccessTokenUrl);
		return AccessToken.fromJson(resultContent);
	}
	
	/**
	 * 发送客服消息 <br/>
	 * 文档位置：发送消息->发送客服消息
	 */
	public static void customSend(String body) {
		System.out.println("customSend body=" + body);
		HttpUtil.doPost(CustomSendUrl, body);
	}

	/**
	 * 发送客服文本消息
	 */
	public static void customSendText(String touser, String content) {
		JSONObject json = new JSONObject();
		json.put("touser", touser);
		json.put("msgtype", "text");
		JSONObject text = new JSONObject();
		text.put("content", content);
		json.put("text", text);
		customSend(json.toString());
	}
	
	/**
	 * 创建自定义菜单<p>
	 * 文档位置：自定义菜单->自定义菜单创建接口
	 */
	public static String menuCreate(String body) {
		return HttpUtil.doPost(CreateMenuUrl, body);
	}

	/**
	 * 查询自定义菜单<p>
	 * 文档位置：自定义菜单->自定义菜单查询接口
	 */
	public static String menuQuery() {
		return HttpUtil.doGet(QueryMenuUrl);
	}
	
	/**
	 * 删除自定义菜单<p>
	 * 文档位置：自定义菜单->自定义菜单删除接口
	 */
	public static String menuDelete() {
		return HttpUtil.doGet(DeleteMenuUrl);
	}

}
