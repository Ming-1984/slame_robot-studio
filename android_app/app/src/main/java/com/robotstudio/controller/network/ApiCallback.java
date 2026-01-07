package com.robotstudio.controller.network;

/**
 * API回调接口
 * @param <T> 成功时返回的数据类型
 */
public interface ApiCallback<T> {
    
    /**
     * 请求成功时调用
     * @param result 返回的数据
     */
    void onSuccess(T result);
    
    /**
     * 请求失败时调用
     * @param error 错误信息
     */
    void onError(String error);
}
