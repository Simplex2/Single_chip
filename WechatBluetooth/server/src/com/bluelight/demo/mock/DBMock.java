package com.bluelight.demo.mock;

import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

/**
 * 模拟 DB 保存用户和设备绑定信息
 * <p>
 * 实际中需要使用数据库持久化
 */
public class DBMock {

	private static Map<String, Map<String, String>> bound = new ConcurrentHashMap<String, Map<String, String>>();

	/**
	 * 根据用户id获取绑定的设备id和设备类型
	 * fromUserName,deviceID,openID,deviceType
	 */
	public static Map<String, String> queryBoundInfo(String fromUserName) {
		Map<String, String> boundInfo = bound.get(fromUserName);
		System.out.println("queryBoundInfoBy=" + boundInfo);
		return boundInfo;
	}

	/**
	 * 保存绑定关系
	 */
	public static void saveBoundInfo(Map<String, String> reqMap) {
		System.out.println("saveBoundInfo=" + reqMap);

		Map<String, String> boundInfo = new HashMap<String, String>();
		boundInfo.put("fromUserName", reqMap.get("FromUserName"));
		boundInfo.put("deviceID", reqMap.get("DeviceID"));
		boundInfo.put("openID", reqMap.get("OpenID"));
		boundInfo.put("deviceType", reqMap.get("DeviceType"));

		bound.put(reqMap.get("FromUserName"), boundInfo);
	}

	/**
	 * 删除绑定关系
	 */
	public static void removeBoundInfo(String fromUserName) {
		System.out.println("removeBoundInfo=" + fromUserName);
		bound.remove(fromUserName);
	}

}
