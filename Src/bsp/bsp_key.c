#include  "bsp/bsp_key.h"

void KEY_GPIO_Init(void)
{
    /* ����IOӲ����ʼ���ṹ����� */
    GPIO_InitTypeDef GPIO_InitStruct;

    /* ʹ��(����)KEY���Ŷ�ӦIO�˿�ʱ�� */
    KEY1_RCC_CLK_ENABLE();
    KEY2_RCC_CLK_ENABLE();

    /* ����KEY1 GPIO:��������ģʽ */
    GPIO_InitStruct.Pin = KEY1_GPIO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(KEY1_GPIO, &GPIO_InitStruct);

    /* ����KEY2 GPIO:��������ģʽ */
    GPIO_InitStruct.Pin = KEY2_GPIO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(KEY2_GPIO, &GPIO_InitStruct);

    /* ����KEY3 GPIO:��������ģʽ */
    GPIO_InitStruct.Pin = KEY3_GPIO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(KEY3_GPIO, &GPIO_InitStruct);

    /* ����KEY3 GPIO:��������ģʽ */
    GPIO_InitStruct.Pin = KEY4_GPIO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(KEY4_GPIO, &GPIO_InitStruct);
}

/**
  * ��������: ��ȡ����KEY1��״̬
  * �����������
  * �� �� ֵ: KEY_DOWN�����������£�
  *           KEY_UP  ������û������
  * ˵    �����ޡ�
  */
KEYState_TypeDef KEY1_StateRead(void)
{
    /* ��ȡ��ʱ����ֵ���ж��Ƿ��Ǳ�����״̬������Ǳ�����״̬���뺯���� */
    if(HAL_GPIO_ReadPin(KEY1_GPIO, KEY1_GPIO_PIN) == KEY1_DOWN_LEVEL)
    {
        /* ��ʱһС��ʱ�䣬�������� */
        HAL_Delay(10);
        /* ��ʱʱ��������жϰ���״̬��������ǰ���״̬˵������ȷʵ������ */
        if(HAL_GPIO_ReadPin(KEY1_GPIO, KEY1_GPIO_PIN) == KEY1_DOWN_LEVEL)
        {
            /* �ȴ������������˳�����ɨ�躯�� */
            while(HAL_GPIO_ReadPin(KEY1_GPIO, KEY1_GPIO_PIN) == KEY1_DOWN_LEVEL);
            /* ����ɨ����ϣ�ȷ�����������£����ذ���������״̬ */
            return KEY_DOWN;
        }
    }
    /* ����û�����£�����û������״̬ */
    return KEY_UP;
}

/**
  * ��������: ��ȡ����KEY2��״̬
  * �����������
  * �� �� ֵ: KEY_DOWN�����������£�
  *           KEY_UP  ������û������
  * ˵    �����ޡ�
  */
KEYState_TypeDef KEY2_StateRead(void)
{
    /* ��ȡ��ʱ����ֵ���ж��Ƿ��Ǳ�����״̬������Ǳ�����״̬���뺯���� */
    if(HAL_GPIO_ReadPin(KEY2_GPIO, KEY2_GPIO_PIN) == KEY2_DOWN_LEVEL)
    {
        /* ��ʱһС��ʱ�䣬�������� */
        HAL_Delay(10);
        /* ��ʱʱ��������жϰ���״̬��������ǰ���״̬˵������ȷʵ������ */
        if(HAL_GPIO_ReadPin(KEY2_GPIO, KEY2_GPIO_PIN) == KEY2_DOWN_LEVEL)
        {
            /* �ȴ������������˳�����ɨ�躯�� */
            while(HAL_GPIO_ReadPin(KEY2_GPIO, KEY2_GPIO_PIN) == KEY2_DOWN_LEVEL);
            /* ����ɨ����ϣ�ȷ�����������£����ذ���������״̬ */
            return KEY_DOWN;
        }
    }
    /* ����û�����£�����û������״̬ */
    return KEY_UP;
}

/**
  * ��������: ��ȡ����KEY2��״̬
  * �����������
  * �� �� ֵ: KEY_DOWN�����������£�
  *           KEY_UP  ������û������
  * ˵    �����ޡ�
  */
KEYState_TypeDef KEY3_StateRead(void)
{
    /* ��ȡ��ʱ����ֵ���ж��Ƿ��Ǳ�����״̬������Ǳ�����״̬���뺯���� */
    if(HAL_GPIO_ReadPin(KEY3_GPIO, KEY3_GPIO_PIN) == KEY3_DOWN_LEVEL)
    {
        /* ��ʱһС��ʱ�䣬�������� */
        HAL_Delay(10);
        /* ��ʱʱ��������жϰ���״̬��������ǰ���״̬˵������ȷʵ������ */
        if(HAL_GPIO_ReadPin(KEY3_GPIO, KEY3_GPIO_PIN) == KEY3_DOWN_LEVEL)
        {
            /* �ȴ������������˳�����ɨ�躯�� */
            //      while(HAL_GPIO_ReadPin(KEY3_GPIO,KEY3_GPIO_PIN)==KEY3_DOWN_LEVEL);
            /* ����ɨ����ϣ�ȷ�����������£����ذ���������״̬ */
            return KEY_DOWN;
        }
    }
    /* ����û�����£�����û������״̬ */
    return KEY_UP;
}

/**
  * ��������: ��ȡ����KEY2��״̬
  * �����������
  * �� �� ֵ: KEY_DOWN�����������£�
  *           KEY_UP  ������û������
  * ˵    �����ޡ�
  */
KEYState_TypeDef KEY4_StateRead(void)
{
    /* ��ȡ��ʱ����ֵ���ж��Ƿ��Ǳ�����״̬������Ǳ�����״̬���뺯���� */
    if(HAL_GPIO_ReadPin(KEY4_GPIO, KEY4_GPIO_PIN) == KEY4_DOWN_LEVEL)
    {
        /* ��ʱһС��ʱ�䣬�������� */
        HAL_Delay(10);
        /* ��ʱʱ��������жϰ���״̬��������ǰ���״̬˵������ȷʵ������ */
        if(HAL_GPIO_ReadPin(KEY4_GPIO, KEY4_GPIO_PIN) == KEY4_DOWN_LEVEL)
        {
            /* �ȴ������������˳�����ɨ�躯�� */
            //      while(HAL_GPIO_ReadPin(KEY3_GPIO,KEY3_GPIO_PIN)==KEY3_DOWN_LEVEL);
            /* ����ɨ����ϣ�ȷ�����������£����ذ���������״̬ */
            return KEY_DOWN;
        }
    }
    /* ����û�����£�����û������״̬ */
    return KEY_UP;
}
