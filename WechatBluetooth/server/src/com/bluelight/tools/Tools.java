package com.bluelight.tools;

import java.io.File;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import net.sf.json.JSONArray;
import net.sf.json.JSONObject;

import com.bluelight.demo.api.DeviceApi;
import com.bluelight.demo.api.MpApi;
import com.bluelight.demo.api.json.DeviceAuth;
import com.bluelight.demo.service.CallbackService;
import com.google.zxing.BarcodeFormat;
import com.google.zxing.EncodeHintType;
import com.google.zxing.MultiFormatWriter;
import com.google.zxing.common.BitMatrix;
import com.google.zxing.qrcode.decoder.ErrorCorrectionLevel;

/**
 * 调试工具类
 * <p>
 * 创建菜单<br/>
 * 获取并生成设备二维码<br/>
 * 设备授权/更新设备属性
 */
@SuppressWarnings("unused")
public class Tools {
	
	//二维码图片存放地址
	static String imagePath = "D:/work/image/";
	
	public static void main(String[] args) throws Exception {

		// 1、申请测试账号
		// 2、在config.properties中配置公众平台账号相关信息及token
		// 3、服务端部署，在测试号中配置服务url和token
		// 4、公众平台开发介绍，服务端代码介绍，创建自定义菜单
		// 5、修改demo服务器代码灯泡通信协议为设备自定义协议
		// 6、确定设备id，生成二维码；确定设备参数，对设备进行授权
		// 7、和设备联调

		if (!new File(imagePath).exists()) {
			System.err.println("路径不存在：" + imagePath);
			return;
		}
		
		//-----------创建菜单
		//createMenu();
		
		// 设备id 由厂商指定，建议由字母、数字、下划线组成，以免json解析失败。
		String deviceId = "gh_3d6e4859690c_199c6b2a70d94916ed95ed6c03acd0f0";
		
		//-----------根据设备id生成二维码，授权后才能进行扫描绑定
		//createQrByDeviceId(deviceId);

		//-----------设备授权 
		// 设备授权后才能进行扫码绑定
		// 开发调试时，可以先用假的信息授权，测试服务端设备绑定事件的处理。
		// 设备参数确定后，更新为正确的设备属性，再和设备联调。
		String authKey = "";		//"1234567890ABCDEF1234567890ABCD11";
		String mac = "f5ccd7ac8760";//"1234567890AB";
		boolean isCreate =  true;	//是否首次授权： true 首次授权； false 更新设备属性
		//device_Auth(authKey, deviceId, mac, isCreate);
	}
	
	/**
	 * 根据deviceId 调用createQrcode生成二维码生成串
	 */
	public static void createQrByDeviceId(String deviceId) throws Exception{
		List<String> list = new ArrayList<String>();
		list.add(deviceId);
		String code = DeviceApi.createQrcode(list);
		// 格式 result={"errcode":0,"errmsg":"ok","device_num":1,"code_list":[{"device_id":"gh_1bafe245c2cb_bluelight_demo_000002","ticket":"http:\/\/we.qq.com\/d\/AQC5v5iz1PEfQTzuo5Ow7U24-pzJwIcfYvZ-y8yS"}]}
		JSONObject codeJson = JSONObject.fromObject(code);
		String ticket = ((JSONArray) codeJson.get("code_list"))
				.getJSONObject(0).getString("ticket");
		System.out.println("ticket=" + ticket);
		
		createQrImage(imagePath , deviceId , ticket );
	}
	
	/**
	 * 使用zxing库生成二维码图片
	 */
	private static void createQrImage(String path, String deviceId,
			String ticket) {
		path = path.endsWith("/") ? path : path + "/";
		int width = 430;
		int height = 430;
		// 二维码的图片格式
		String format = "jpg";
		String fileName = path + deviceId + "."+ format;
		// 设置二维码的参数
		Map<EncodeHintType, Object> hints = new HashMap<EncodeHintType, Object>();
		hints.put(EncodeHintType.CHARACTER_SET, "utf-8");//编码
		hints.put(EncodeHintType.ERROR_CORRECTION, ErrorCorrectionLevel.Q);//容错率
		try {
			// 生成二维码
			BitMatrix bitMatrix = new MultiFormatWriter().encode(ticket,
					BarcodeFormat.QR_CODE, width, height, hints);
			// 输出图片
			File outputFile = new File(fileName);
			MatrixToImageWriter.writeToFile(bitMatrix, format, outputFile);

			System.out.println("设备id：" + deviceId + "，ticket："+  ticket + "，生成二维码图片：" + fileName);
		} catch (Exception e) {
			e.printStackTrace();
		}
	}
	
	/**
	 * 设备授权
	 * @param authKey 加密key
	 * @param deviceId 设备id
	 * @param mac 设备的mac地址
	 * @param 是否首次授权： true 首次授权； false 更新设备属性
	 */
	public static void device_Auth(String authKey, String deviceId, String mac,
			boolean isCreate) throws Exception {
		DeviceAuth device = new DeviceAuth();
		device.setId(deviceId);		//设备id
		device.setMac(mac);			//设备的mac地址 采用16进制串的方式（长度为12字节），不需要0X前缀，如： 1234567890AB
		device.setConnect_protocol("3");//设备类型 android classic bluetooth – 1 ios classic bluetooth – 2 ble – 3 wifi -- 4
		
		// 不加密时 authKey 为空字符串，crypt_method、auth_ver都为0
		// 加密时 authKey 需为符合格式的值，crypt_method、auth_ver都为1
		device.setAuth_key(authKey);	//加密key 1234567890ABCDEF1234567890ABCDEF
		device.setCrypt_method("0");    //auth加密方法  0：不加密 1：AES加密
		device.setAuth_ver("0");        //0：不加密的version 1：version 1
		
		/**
		 * 连接策略，32位整型，按bit位置位，目前仅第1bit和第3bit位有效（bit置0为无效，1为有效；第2bit已被废弃），且bit位可以按或置位
		 * （如1|4=5），各bit置位含义说明如下：<br/>
		 * 1：（第1bit置位）在公众号对话页面，不停的尝试连接设备<br/>
		 * 4：（第3bit置位）处于非公众号页面（如主界面等），微信自动连接。当用户切换微信到前台时，可能尝试去连接设备，连上后一定时间会断开<br/>
		 * 8：（第4bit置位），进入微信后即刻开始连接。只要微信进程在运行就不会主动断开
		 */
		device.setConn_strategy("1");   //连接策略
		device.setClose_strategy("2");  //1：退出公众号页面时断开 2：退出公众号之后保持连接不断开 3：一直保持连接（设备主动断开连接后，微信尝试重连）

		// 低功耗蓝牙必须为-1
		device.setManu_mac_pos("-1");   //表示mac地址在厂商广播manufature data里含有mac地址的偏移，取值如下： -1：在尾部、 -2：表示不包含mac地址
		// 
		device.setSer_mac_pos("-2");    //表示mac地址在厂商serial number里含有mac地址的偏移，取值如下： -1：表示在尾部 -2：表示不包含mac地址 其他：非法偏移
		
		// 调用授权
		List<DeviceAuth> auths = new ArrayList<DeviceAuth>();
		auths.add(device);
		System.out.println(DeviceApi.authorize(auths,isCreate));
		// {"resp":[{"base_info":{"device_type":"gh_1bafe245c2cb","device_id":
		// "gh_1bafe245c2cb_9e081608d6d62b984edf52d5d3a50aba"},"errcode":0,"errmsg":"ok"}]}
	}
	
	/**
	 * 创建自定义菜单
	 */
	public static void createMenu() throws Exception {
		// 创建菜单
		JSONObject buttonOn = new JSONObject();
		buttonOn.put("type", "click");
		buttonOn.put("name", "点灯");
		buttonOn.put("key", CallbackService.V1001_LIGHT_ON);

		JSONObject buttonOff = new JSONObject();
		buttonOff.put("type", "click");
		buttonOff.put("name", "灭灯");
		buttonOff.put("key", CallbackService.V1002_LIGHT_OFF);

		JSONArray buttons = new JSONArray();
		buttons.add(buttonOn);
		buttons.add(buttonOff);

		JSONObject menu = new JSONObject();
		menu.put("button", buttons);

		System.out.println("菜单：" + menu.toString());
		System.out.println("创建菜单返回：" + MpApi.menuCreate(menu.toString()));
		System.out.println("查询菜单：" + MpApi.menuQuery());
	}

}
