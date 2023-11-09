# RFID-RC522

## Initialization

Before using the library it's necessary to initialize it first and to get the handle.

```C
mfrc_handle* mfrc522_init(itf_write_ptr fptr_write, itf_read_ptr fptr_read, itf_pwr fptr_set_pwr)
```

Init function takes 3 arguments:

- `fptr_write` of type `int (*itf_write_ptr)(size_t, unsigned char [])`
- `fptr_read` of type `int (*itf_read_ptr)(size_t, unsigned char [])`
- `fptr_set_pwr` of type `void (*itf_pwr)(bool)`

These functions must be provided by the caller, depending of what type of communication is used. 
After successfull initialization `mfrc522_pwr_on()` must be called, otherwise the module won't work, although no error will be returned .

### Platform dependent functions

`itf_write_ptr` takes 2 arguments, `length` of the data array and data array (which must be of the same length). 

`itf_read_ptr` takes 2 arguments, `length` of the data array, whch is used for both writing from and writing to. So the addresses of the registers should be provided in that array. After reading all of them, it also should be used as the return array.

`itf_pwr` take 1 argument, whether the power should be turned on(`true`) or off(`false`). It is used as the hardware switch.


#### Example (for SPI communication on STM32)

`RC522_SS_GPIO_Port` and `RC522_SS_Pin` are defines for GPIO pin for NSS (software).

`RC522_RST_GPIO_Port` and `RC522_RST_Pin` are defines for GPIO pin for hardware reset of the module.

```C
int spi_write(size_t len, unsigned char data_in_out[static len])
{
	HAL_GPIO_WritePin(RC522_SS_GPIO_Port, RC522_SS_Pin, GPIO_PIN_RESET);
	HAL_StatusTypeDef result = HAL_SPI_Transmit(&hspi1, data_in_out, len, 100);
	HAL_GPIO_WritePin(RC522_SS_GPIO_Port, RC522_SS_Pin, GPIO_PIN_SET);
	if(result != HAL_OK){
		return -1;
	}
	return 0;
}

int spi_read(size_t len, unsigned char data_in_out[static len])
{
	unsigned char ret[len];

	HAL_GPIO_WritePin(RC522_SS_GPIO_Port, RC522_SS_Pin, GPIO_PIN_RESET);
	HAL_StatusTypeDef result = HAL_SPI_TransmitReceive(&hspi1, data_in_out, ret, len, 100);
	HAL_GPIO_WritePin(RC522_SS_GPIO_Port, RC522_SS_Pin, GPIO_PIN_SET);
	if(result != HAL_OK){
		return -1;
	}
	for(size_t i = 1; i < len; i++)
	{
		data_in_out[i] = ret[i];
	}
	return 0;
}
void set_pwr(bool powered_on)
{
	HAL_GPIO_WritePin(RC522_RST_GPIO_Port, RC522_RST_Pin, powered_on ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_Delay(10);
}
```


