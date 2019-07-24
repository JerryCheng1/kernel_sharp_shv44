/* Copyright (c) 2017-2018, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/crc32.h>
#include <media/cam_sensor.h>

#include "cam_eeprom_core.h"
#include "cam_eeprom_soc.h"
#include "cam_debug_util.h"
#include "cam_common_util.h"
#include "cam_packet_util.h"

/**
 * cam_eeprom_read_memory() - read map data into buffer
 * @e_ctrl:     eeprom control struct
 * @block:      block to be read
 *
 * This function iterates through blocks stored in block->map, reads each
 * region and concatenate them into the pre-allocated block->mapdata
 */
static int cam_eeprom_read_memory(struct cam_eeprom_ctrl_t *e_ctrl,
	struct cam_eeprom_memory_block_t *block)
{
	int                                rc = 0;
	int                                j;
	struct cam_sensor_i2c_reg_setting  i2c_reg_settings = {0};
	struct cam_sensor_i2c_reg_array    i2c_reg_array = {0};
	struct cam_eeprom_memory_map_t    *emap = block->map;
	struct cam_eeprom_soc_private     *eb_info = NULL;
	uint8_t                           *memptr = block->mapdata;

	if (!e_ctrl) {
		CAM_ERR(CAM_EEPROM, "e_ctrl is NULL");
		return -EINVAL;
	}

	eb_info = (struct cam_eeprom_soc_private *)e_ctrl->soc_info.soc_private;

	for (j = 0; j < block->num_map; j++) {
		CAM_DBG(CAM_EEPROM, "slave-addr = 0x%X", emap[j].saddr);
		if (emap[j].saddr) {
			eb_info->i2c_info.slave_addr = emap[j].saddr;
			rc = cam_eeprom_update_i2c_info(e_ctrl,
				&eb_info->i2c_info);
			if (rc) {
				CAM_ERR(CAM_EEPROM,
					"failed: to update i2c info rc %d",
					rc);
				return rc;
			}
		}

		if (emap[j].page.valid_size) {
			i2c_reg_settings.addr_type = emap[j].page.addr_type;
			i2c_reg_settings.data_type = emap[j].page.data_type;
			i2c_reg_settings.size = 1;
			i2c_reg_array.reg_addr = emap[j].page.addr;
			i2c_reg_array.reg_data = emap[j].page.data;
			i2c_reg_array.delay = emap[j].page.delay;
			i2c_reg_settings.reg_setting = &i2c_reg_array;
			rc = camera_io_dev_write(&e_ctrl->io_master_info,
				&i2c_reg_settings);
			if (rc) {
				CAM_ERR(CAM_EEPROM, "page write failed rc %d",
					rc);
				return rc;
			}
		}

		if (emap[j].pageen.valid_size) {
			i2c_reg_settings.addr_type = emap[j].pageen.addr_type;
			i2c_reg_settings.data_type = emap[j].pageen.data_type;
			i2c_reg_settings.size = 1;
			i2c_reg_array.reg_addr = emap[j].pageen.addr;
			i2c_reg_array.reg_data = emap[j].pageen.data;
			i2c_reg_array.delay = emap[j].pageen.delay;
			i2c_reg_settings.reg_setting = &i2c_reg_array;
			rc = camera_io_dev_write(&e_ctrl->io_master_info,
				&i2c_reg_settings);
			if (rc) {
				CAM_ERR(CAM_EEPROM, "page enable failed rc %d",
					rc);
				return rc;
			}
		}

		if (emap[j].poll.valid_size) {
			rc = camera_io_dev_poll(&e_ctrl->io_master_info,
				emap[j].poll.addr, emap[j].poll.data,
				0, emap[j].poll.addr_type,
				emap[j].poll.data_type,
				emap[j].poll.delay);
			if (rc) {
				CAM_ERR(CAM_EEPROM, "poll failed rc %d",
					rc);
				return rc;
			}
		}

		if (emap[j].mem.valid_size) {
			rc = camera_io_dev_read_seq(&e_ctrl->io_master_info,
				emap[j].mem.addr, memptr,
				emap[j].mem.addr_type,
				emap[j].mem.data_type,
				emap[j].mem.valid_size);
			if (rc) {
				CAM_ERR(CAM_EEPROM, "read failed rc %d",
					rc);
				return rc;
			}
			memptr += emap[j].mem.valid_size;
		}

		if (emap[j].pageen.valid_size) {
			i2c_reg_settings.addr_type = emap[j].pageen.addr_type;
			i2c_reg_settings.data_type = emap[j].pageen.data_type;
			i2c_reg_settings.size = 1;
			i2c_reg_array.reg_addr = emap[j].pageen.addr;
			i2c_reg_array.reg_data = 0;
			i2c_reg_array.delay = emap[j].pageen.delay;
			i2c_reg_settings.reg_setting = &i2c_reg_array;
			rc = camera_io_dev_write(&e_ctrl->io_master_info,
				&i2c_reg_settings);
			if (rc) {
				CAM_ERR(CAM_EEPROM,
					"page disable failed rc %d",
					rc);
				return rc;
			}
		}
	}
	return rc;
}

/**
 * cam_eeprom_power_up - Power up eeprom hardware
 * @e_ctrl:     ctrl structure
 * @power_info: power up/down info for eeprom
 *
 * Returns success or failure
 */
static int cam_eeprom_power_up(struct cam_eeprom_ctrl_t *e_ctrl,
	struct cam_sensor_power_ctrl_t *power_info)
{
	int32_t                 rc = 0;
	struct cam_hw_soc_info *soc_info =
		&e_ctrl->soc_info;

	/* Parse and fill vreg params for power up settings */
	rc = msm_camera_fill_vreg_params(
		&e_ctrl->soc_info,
		power_info->power_setting,
		power_info->power_setting_size);
	if (rc) {
		CAM_ERR(CAM_EEPROM,
			"failed to fill power up vreg params rc:%d", rc);
		return rc;
	}

	/* Parse and fill vreg params for power down settings*/
	rc = msm_camera_fill_vreg_params(
		&e_ctrl->soc_info,
		power_info->power_down_setting,
		power_info->power_down_setting_size);
	if (rc) {
		CAM_ERR(CAM_EEPROM,
			"failed to fill power down vreg params  rc:%d", rc);
		return rc;
	}

	power_info->dev = soc_info->dev;

	rc = cam_sensor_core_power_up(power_info, soc_info);
	if (rc) {
		CAM_ERR(CAM_EEPROM, "failed in eeprom power up rc %d", rc);
		return rc;
	}

	if (e_ctrl->io_master_info.master_type == CCI_MASTER) {
		rc = camera_io_init(&(e_ctrl->io_master_info));
		if (rc) {
			CAM_ERR(CAM_EEPROM, "cci_init failed");
			return -EINVAL;
		}
	}
	return rc;
}

/**
 * cam_eeprom_power_down - Power down eeprom hardware
 * @e_ctrl:    ctrl structure
 *
 * Returns success or failure
 */
static int cam_eeprom_power_down(struct cam_eeprom_ctrl_t *e_ctrl)
{
	struct cam_sensor_power_ctrl_t *power_info;
	struct cam_hw_soc_info         *soc_info;
	struct cam_eeprom_soc_private  *soc_private;
	int                             rc = 0;

	if (!e_ctrl) {
		CAM_ERR(CAM_EEPROM, "failed: e_ctrl %pK", e_ctrl);
		return -EINVAL;
	}

	soc_private =
		(struct cam_eeprom_soc_private *)e_ctrl->soc_info.soc_private;
	power_info = &soc_private->power_info;
	soc_info = &e_ctrl->soc_info;

	if (!power_info) {
		CAM_ERR(CAM_EEPROM, "failed: power_info %pK", power_info);
		return -EINVAL;
	}
	rc = cam_sensor_util_power_down(power_info, soc_info);
	if (rc) {
		CAM_ERR(CAM_EEPROM, "power down the core is failed:%d", rc);
		return rc;
	}

	if (e_ctrl->io_master_info.master_type == CCI_MASTER)
		camera_io_release(&(e_ctrl->io_master_info));

	return rc;
}

/**
 * cam_eeprom_match_id - match eeprom id
 * @e_ctrl:     ctrl structure
 *
 * Returns success or failure
 */
static int cam_eeprom_match_id(struct cam_eeprom_ctrl_t *e_ctrl)
{
	int                      rc;
	struct camera_io_master *client = &e_ctrl->io_master_info;
	uint8_t                  id[2];

	rc = cam_spi_query_id(client, 0, CAMERA_SENSOR_I2C_TYPE_WORD,
		&id[0], 2);
	if (rc)
		return rc;
	CAM_DBG(CAM_EEPROM, "read 0x%x 0x%x, check 0x%x 0x%x",
		id[0], id[1], client->spi_client->mfr_id0,
		client->spi_client->device_id0);
	if (id[0] != client->spi_client->mfr_id0
		|| id[1] != client->spi_client->device_id0)
		return -ENODEV;
	return 0;
}

/**
 * cam_eeprom_parse_read_memory_map - Parse memory map
 * @of_node:    device node
 * @e_ctrl:     ctrl structure
 *
 * Returns success or failure
 */
int32_t cam_eeprom_parse_read_memory_map(struct device_node *of_node,
	struct cam_eeprom_ctrl_t *e_ctrl)
{
	int32_t                         rc = 0;
	struct cam_eeprom_soc_private  *soc_private;
	struct cam_sensor_power_ctrl_t *power_info;

	if (!e_ctrl) {
		CAM_ERR(CAM_EEPROM, "failed: e_ctrl is NULL");
		return -EINVAL;
	}

	soc_private =
		(struct cam_eeprom_soc_private *)e_ctrl->soc_info.soc_private;
	power_info = &soc_private->power_info;

	rc = cam_eeprom_parse_dt_memory_map(of_node, &e_ctrl->cal_data);
	if (rc) {
		CAM_ERR(CAM_EEPROM, "failed: eeprom dt parse rc %d", rc);
		return rc;
	}
	rc = cam_eeprom_power_up(e_ctrl, power_info);
	if (rc) {
		CAM_ERR(CAM_EEPROM, "failed: eeprom power up rc %d", rc);
		goto data_mem_free;
	}

	e_ctrl->cam_eeprom_state = CAM_EEPROM_CONFIG;
	if (e_ctrl->eeprom_device_type == MSM_CAMERA_SPI_DEVICE) {
		rc = cam_eeprom_match_id(e_ctrl);
		if (rc) {
			CAM_DBG(CAM_EEPROM, "eeprom not matching %d", rc);
			goto power_down;
		}
	}
	rc = cam_eeprom_read_memory(e_ctrl, &e_ctrl->cal_data);
	if (rc) {
		CAM_ERR(CAM_EEPROM, "read_eeprom_memory failed");
		goto power_down;
	}

	rc = cam_eeprom_power_down(e_ctrl);
	if (rc)
		CAM_ERR(CAM_EEPROM, "failed: eeprom power down rc %d", rc);

	e_ctrl->cam_eeprom_state = CAM_EEPROM_ACQUIRE;
	return rc;
power_down:
	cam_eeprom_power_down(e_ctrl);
data_mem_free:
	vfree(e_ctrl->cal_data.mapdata);
	vfree(e_ctrl->cal_data.map);
	e_ctrl->cal_data.num_data = 0;
	e_ctrl->cal_data.num_map = 0;
	e_ctrl->cam_eeprom_state = CAM_EEPROM_ACQUIRE;
	return rc;
}

/**
 * cam_eeprom_get_dev_handle - get device handle
 * @e_ctrl:     ctrl structure
 * @arg:        Camera control command argument
 *
 * Returns success or failure
 */
static int32_t cam_eeprom_get_dev_handle(struct cam_eeprom_ctrl_t *e_ctrl,
	void *arg)
{
	struct cam_sensor_acquire_dev    eeprom_acq_dev;
	struct cam_create_dev_hdl        bridge_params;
	struct cam_control              *cmd = (struct cam_control *)arg;

	if (e_ctrl->bridge_intf.device_hdl != -1) {
		CAM_ERR(CAM_EEPROM, "Device is already acquired");
		return -EFAULT;
	}
	if (copy_from_user(&eeprom_acq_dev,
		u64_to_user_ptr(cmd->handle),
		sizeof(eeprom_acq_dev))) {
		CAM_ERR(CAM_EEPROM,
			"EEPROM:ACQUIRE_DEV: copy from user failed");
		return -EFAULT;
	}

	bridge_params.session_hdl = eeprom_acq_dev.session_handle;
	bridge_params.ops = &e_ctrl->bridge_intf.ops;
	bridge_params.v4l2_sub_dev_flag = 0;
	bridge_params.media_entity_flag = 0;
	bridge_params.priv = e_ctrl;

	eeprom_acq_dev.device_handle =
		cam_create_device_hdl(&bridge_params);
	e_ctrl->bridge_intf.device_hdl = eeprom_acq_dev.device_handle;
	e_ctrl->bridge_intf.session_hdl = eeprom_acq_dev.session_handle;

	CAM_DBG(CAM_EEPROM, "Device Handle: %d", eeprom_acq_dev.device_handle);
	if (copy_to_user(u64_to_user_ptr(cmd->handle),
		&eeprom_acq_dev, sizeof(struct cam_sensor_acquire_dev))) {
		CAM_ERR(CAM_EEPROM, "EEPROM:ACQUIRE_DEV: copy to user failed");
		return -EFAULT;
	}
	return 0;
}

/**
 * cam_eeprom_update_slaveInfo - Update slave info
 * @e_ctrl:     ctrl structure
 * @cmd_buf:    command buffer
 *
 * Returns success or failure
 */
static int32_t cam_eeprom_update_slaveInfo(struct cam_eeprom_ctrl_t *e_ctrl,
	void *cmd_buf)
{
	int32_t                         rc = 0;
	struct cam_eeprom_soc_private  *soc_private;
	struct cam_cmd_i2c_info        *cmd_i2c_info = NULL;

	soc_private =
		(struct cam_eeprom_soc_private *)e_ctrl->soc_info.soc_private;
	cmd_i2c_info = (struct cam_cmd_i2c_info *)cmd_buf;
	soc_private->i2c_info.slave_addr = cmd_i2c_info->slave_addr;
	soc_private->i2c_info.i2c_freq_mode = cmd_i2c_info->i2c_freq_mode;

	rc = cam_eeprom_update_i2c_info(e_ctrl,
		&soc_private->i2c_info);
	CAM_DBG(CAM_EEPROM, "Slave addr: 0x%x Freq Mode: %d",
		soc_private->i2c_info.slave_addr,
		soc_private->i2c_info.i2c_freq_mode);

	return rc;
}

/**
 * cam_eeprom_parse_memory_map - Parse memory map info
 * @data:             memory block data
 * @cmd_buf:          command buffer
 * @cmd_length:       command buffer length
 * @num_map:          memory map size
 * @cmd_length_bytes: command length processed in this function
 *
 * Returns success or failure
 */
static int32_t cam_eeprom_parse_memory_map(
	struct cam_eeprom_memory_block_t *data,
	void *cmd_buf, int cmd_length, uint16_t *cmd_length_bytes,
	int *num_map, size_t remain_buf_len)
{
	int32_t                            rc = 0;
	int32_t                            cnt = 0;
	int32_t                            processed_size = 0;
	uint8_t                            generic_op_code;
	struct cam_eeprom_memory_map_t    *map = data->map;
	struct common_header              *cmm_hdr =
		(struct common_header *)cmd_buf;
	uint16_t                           cmd_length_in_bytes = 0;
	struct cam_cmd_i2c_random_wr      *i2c_random_wr = NULL;
	struct cam_cmd_i2c_continuous_rd  *i2c_cont_rd = NULL;
	struct cam_cmd_conditional_wait   *i2c_poll = NULL;
	struct cam_cmd_unconditional_wait *i2c_uncond_wait = NULL;
	size_t                             validate_size = 0;

	generic_op_code = cmm_hdr->third_byte;

	if (cmm_hdr->cmd_type == CAMERA_SENSOR_CMD_TYPE_I2C_RNDM_WR)
		validate_size = sizeof(struct cam_cmd_i2c_random_wr);
	else if (cmm_hdr->cmd_type == CAMERA_SENSOR_CMD_TYPE_I2C_CONT_RD)
		validate_size = sizeof(struct cam_cmd_i2c_continuous_rd);
	else if (cmm_hdr->cmd_type == CAMERA_SENSOR_CMD_TYPE_WAIT)
		validate_size = sizeof(struct cam_cmd_unconditional_wait);

	if (remain_buf_len < validate_size) {
		CAM_ERR(CAM_EEPROM, "not enough buffer");
		return -EINVAL;
	}
	switch (cmm_hdr->cmd_type) {
	case CAMERA_SENSOR_CMD_TYPE_I2C_RNDM_WR:
		i2c_random_wr = (struct cam_cmd_i2c_random_wr *)cmd_buf;
		cmd_length_in_bytes   = sizeof(struct cam_cmd_i2c_random_wr) +
			((i2c_random_wr->header.count - 1) *
			sizeof(struct i2c_random_wr_payload));

		for (cnt = 0; cnt < (i2c_random_wr->header.count);
			cnt++) {
			map[*num_map + cnt].page.addr =
				i2c_random_wr->random_wr_payload[cnt].reg_addr;
			map[*num_map + cnt].page.addr_type =
				i2c_random_wr->header.addr_type;
			map[*num_map + cnt].page.data =
				i2c_random_wr->random_wr_payload[cnt].reg_data;
			map[*num_map + cnt].page.data_type =
				i2c_random_wr->header.data_type;
			map[*num_map + cnt].page.valid_size = 1;
		}

		*num_map += (i2c_random_wr->header.count - 1);
		cmd_buf += cmd_length_in_bytes / sizeof(int32_t);
		processed_size +=
			cmd_length_in_bytes;
		break;
	case CAMERA_SENSOR_CMD_TYPE_I2C_CONT_RD:
		i2c_cont_rd = (struct cam_cmd_i2c_continuous_rd *)cmd_buf;
		cmd_length_in_bytes = sizeof(struct cam_cmd_i2c_continuous_rd);

		map[*num_map].mem.addr = i2c_cont_rd->reg_addr;
		map[*num_map].mem.addr_type = i2c_cont_rd->header.addr_type;
		map[*num_map].mem.data_type = i2c_cont_rd->header.data_type;
		map[*num_map].mem.valid_size =
			i2c_cont_rd->header.count;
		cmd_buf += cmd_length_in_bytes / sizeof(int32_t);
		processed_size +=
			cmd_length_in_bytes;
		data->num_data += map[*num_map].mem.valid_size;
		break;
	case CAMERA_SENSOR_CMD_TYPE_WAIT:
		if (generic_op_code ==
			CAMERA_SENSOR_WAIT_OP_HW_UCND ||
			generic_op_code ==
			CAMERA_SENSOR_WAIT_OP_SW_UCND) {
			i2c_uncond_wait =
				(struct cam_cmd_unconditional_wait *)cmd_buf;
			cmd_length_in_bytes =
				sizeof(struct cam_cmd_unconditional_wait);

			if (*num_map < 1) {
				CAM_ERR(CAM_EEPROM,
					"invalid map number, num_map=%d",
					*num_map);
				return -EINVAL;
			}

			/*
			 * Though delay is added all of them, but delay will
			 * be applicable to only one of them as only one of
			 * them will have valid_size set to >= 1.
			 */
			map[*num_map - 1].mem.delay = i2c_uncond_wait->delay;
			map[*num_map - 1].page.delay = i2c_uncond_wait->delay;
			map[*num_map - 1].pageen.delay = i2c_uncond_wait->delay;
		} else if (generic_op_code ==
			CAMERA_SENSOR_WAIT_OP_COND) {
			i2c_poll = (struct cam_cmd_conditional_wait *)cmd_buf;
			cmd_length_in_bytes =
				sizeof(struct cam_cmd_conditional_wait);

			map[*num_map].poll.addr = i2c_poll->reg_addr;
			map[*num_map].poll.addr_type = i2c_poll->addr_type;
			map[*num_map].poll.data = i2c_poll->reg_data;
			map[*num_map].poll.data_type = i2c_poll->data_type;
			map[*num_map].poll.delay = i2c_poll->timeout;
			map[*num_map].poll.valid_size = 1;
		}
		cmd_buf += cmd_length_in_bytes / sizeof(int32_t);
		processed_size +=
			cmd_length_in_bytes;
		break;
	default:
		break;
	}

	*cmd_length_bytes = processed_size;
	return rc;
}

/**
 * cam_eeprom_init_pkt_parser - Parse eeprom packet
 * @e_ctrl:       ctrl structure
 * @csl_packet:	  csl packet received
 *
 * Returns success or failure
 */
static int32_t cam_eeprom_init_pkt_parser(struct cam_eeprom_ctrl_t *e_ctrl,
	struct cam_packet *csl_packet)
{
	int32_t                         rc = 0;
	int                             i = 0;
	struct cam_cmd_buf_desc        *cmd_desc = NULL;
	uint32_t                       *offset = NULL;
	uint32_t                       *cmd_buf = NULL;
	uintptr_t                        generic_pkt_addr;
	size_t                          pkt_len = 0;
	size_t                          remain_len = 0;
	uint32_t                        total_cmd_buf_in_bytes = 0;
	uint32_t                        processed_cmd_buf_in_bytes = 0;
	struct common_header           *cmm_hdr = NULL;
	uint16_t                        cmd_length_in_bytes = 0;
	struct cam_cmd_i2c_info        *i2c_info = NULL;
	int                             num_map = -1;
	struct cam_eeprom_memory_map_t *map = NULL;
	struct cam_eeprom_soc_private  *soc_private =
		(struct cam_eeprom_soc_private *)e_ctrl->soc_info.soc_private;
	struct cam_sensor_power_ctrl_t *power_info = &soc_private->power_info;

	e_ctrl->cal_data.map = vzalloc((MSM_EEPROM_MEMORY_MAP_MAX_SIZE *
		MSM_EEPROM_MAX_MEM_MAP_CNT) *
		(sizeof(struct cam_eeprom_memory_map_t)));
	if (!e_ctrl->cal_data.map) {
		rc = -ENOMEM;
		CAM_ERR(CAM_EEPROM, "failed");
		return rc;
	}
	map = e_ctrl->cal_data.map;

	offset = (uint32_t *)&csl_packet->payload;
	offset += (csl_packet->cmd_buf_offset / sizeof(uint32_t));
	cmd_desc = (struct cam_cmd_buf_desc *)(offset);

	/* Loop through multiple command buffers */
	for (i = 0; i < csl_packet->num_cmd_buf; i++) {
		total_cmd_buf_in_bytes = cmd_desc[i].length;
		processed_cmd_buf_in_bytes = 0;
		if (!total_cmd_buf_in_bytes)
			continue;
		rc = cam_mem_get_cpu_buf(cmd_desc[i].mem_handle,
			&generic_pkt_addr, &pkt_len);
		if (rc) {
			CAM_ERR(CAM_EEPROM, "Failed to get cpu buf");
			return rc;
		}
		cmd_buf = (uint32_t *)generic_pkt_addr;
		if (!cmd_buf) {
			CAM_ERR(CAM_EEPROM, "invalid cmd buf");
			rc = -EINVAL;
			goto rel_cmd_buf;
		}

		if ((pkt_len < sizeof(struct common_header)) ||
			(cmd_desc[i].offset > (pkt_len -
			sizeof(struct common_header)))) {
			CAM_ERR(CAM_EEPROM, "Not enough buffer");
			rc = -EINVAL;
			goto rel_cmd_buf;
		}
		remain_len = pkt_len - cmd_desc[i].offset;
		cmd_buf += cmd_desc[i].offset / sizeof(uint32_t);

		if (total_cmd_buf_in_bytes > remain_len) {
			CAM_ERR(CAM_EEPROM, "Not enough buffer for command");
			rc = -EINVAL;
			goto rel_cmd_buf;
		}
		/* Loop through multiple cmd formats in one cmd buffer */
		while (processed_cmd_buf_in_bytes < total_cmd_buf_in_bytes) {
			if ((remain_len - processed_cmd_buf_in_bytes) <
				sizeof(struct common_header)) {
				CAM_ERR(CAM_EEPROM, "Not enough buf");
				rc = -EINVAL;
				goto rel_cmd_buf;
			}
			cmm_hdr = (struct common_header *)cmd_buf;
			switch (cmm_hdr->cmd_type) {
			case CAMERA_SENSOR_CMD_TYPE_I2C_INFO:
				i2c_info = (struct cam_cmd_i2c_info *)cmd_buf;
				if ((remain_len - processed_cmd_buf_in_bytes) <
					sizeof(struct cam_cmd_i2c_info)) {
					CAM_ERR(CAM_EEPROM, "Not enough buf");
					rc = -EINVAL;
					goto rel_cmd_buf;
				}
				/* Configure the following map slave address */
				map[num_map + 1].saddr = i2c_info->slave_addr;
				rc = cam_eeprom_update_slaveInfo(e_ctrl,
					cmd_buf);
				cmd_length_in_bytes =
					sizeof(struct cam_cmd_i2c_info);
				processed_cmd_buf_in_bytes +=
					cmd_length_in_bytes;
				cmd_buf += cmd_length_in_bytes/
					sizeof(uint32_t);
				break;
			case CAMERA_SENSOR_CMD_TYPE_PWR_UP:
			case CAMERA_SENSOR_CMD_TYPE_PWR_DOWN:
				cmd_length_in_bytes = total_cmd_buf_in_bytes;
				rc = cam_sensor_update_power_settings(cmd_buf,
					cmd_length_in_bytes, power_info,
					(remain_len -
					processed_cmd_buf_in_bytes));
				processed_cmd_buf_in_bytes +=
					cmd_length_in_bytes;
				cmd_buf += cmd_length_in_bytes/
					sizeof(uint32_t);
				if (rc) {
					CAM_ERR(CAM_EEPROM, "Failed");
					goto rel_cmd_buf;
				}
				break;
			case CAMERA_SENSOR_CMD_TYPE_I2C_RNDM_WR:
			case CAMERA_SENSOR_CMD_TYPE_I2C_CONT_RD:
			case CAMERA_SENSOR_CMD_TYPE_WAIT:
				num_map++;
				rc = cam_eeprom_parse_memory_map(
					&e_ctrl->cal_data, cmd_buf,
					total_cmd_buf_in_bytes,
					&cmd_length_in_bytes, &num_map,
					(remain_len -
					processed_cmd_buf_in_bytes));
				processed_cmd_buf_in_bytes +=
					cmd_length_in_bytes;
				cmd_buf += cmd_length_in_bytes/sizeof(uint32_t);
				break;
			default:
				break;
			}
		}
		e_ctrl->cal_data.num_map = num_map + 1;
		if (cam_mem_put_cpu_buf(cmd_desc[i].mem_handle))
			CAM_WARN(CAM_EEPROM, "Failed to put cpu buf: 0x%x",
				cmd_desc[i].mem_handle);
	}

	return rc;

rel_cmd_buf:
	if (cam_mem_put_cpu_buf(cmd_desc[i].mem_handle))
		CAM_WARN(CAM_EEPROM, "Failed to put cpu buf: 0x%x",
			cmd_desc[i].mem_handle);

	return rc;
}

/* SHLOCAL_CAMERA_IMAGE_QUALITY-> *//* lsc for imx563/imx476/imx351 */
#define MIN(x, y) (((x) > (y)) ? (y):(x))
#define MAX(x, y) (((x) > (y)) ? (x):(y))
#define IMX476_OTP_LSC_W    (17)
#define IMX476_OTP_LSC_H    (13)
#define IMX476_OTP_LSC_SIZE (IMX476_OTP_LSC_W * IMX476_OTP_LSC_H)
#define IMX476_OTP_LSC_BUF_SIZE 96
#define IMX351_OTP_LSC_W    (9)
#define IMX351_OTP_LSC_H    (7)
#define IMX351_OTP_LSC_SIZE (IMX351_OTP_LSC_W * IMX351_OTP_LSC_H)
#define MESH_ROLLOFF_W      (17)
#define MESH_ROLLOFF_H      (13)
#define MESH_ROLLOFF_SIZE   (MESH_ROLLOFF_W * MESH_ROLLOFF_H)

#define IMX563_OTP_WB_OFFSET         (0x10)
#define IMX563_OTP_VCMOIS_OFFSET     (0x40)
#define IMX563_OTP_VCMAF_OFFSET      (0x90)
#define IMX563_OTP_LSC_OFFSET        (0x100)
#define IMX563_OTP_LRC_OFFSET        (0xF00)
#define IMX563_OTP_DCC_OFFSET        (0x1280)

#define IMX476_OTP_WB_OFFSET         (0x8)
#define IMX476_OTP_LSC_OFFSET        (0x10)
#define IMX476_OTP_QSC_OFFSET        (0x700)
#define IMX476_OTP_DPC_OFFSET        (0x0B20)
#define IMX476_OTP_LSC_QSC_OFFSET    (0x1310)

#define IMX351_OTP_WB_OFFSET         (0x10)
#define IMX351_OTP_LSC_OFFSET        (0x40)
#define IMX351_OTP_EXLSC_OFFSET      (0x240) /* >= 576 */


#if 0 /* reject OTP error module */
#if 1 /* Check LSC irregular value */
#define CHECK_SPEC   100
#define H_MAX        MESH_ROLLOFF_W
#define V_MAX        MESH_ROLLOFF_H

static uint16_t otp_lsc_dummy[][221] = {
	{ /* R */
		125, 162, 194, 232, 271, 304, 339, 361, 372, 364, 344, 317, 280, 242, 201, 165, 130,
		151, 188, 234, 282, 335, 387, 436, 473, 489, 480, 447, 399, 345, 293, 242, 196, 156,
		168, 217, 270, 334, 403, 483, 556, 603, 625, 610, 566, 499, 421, 348, 284, 226, 175,
		187, 244, 306, 386, 481, 584, 671, 733, 760, 745, 691, 603, 503, 403, 322, 256, 195,
		203, 265, 338, 434, 548, 667, 774, 851, 887, 867, 798, 694, 574, 454, 354, 280, 212,
		214, 280, 359, 468, 598, 729, 851, 942, 981, 955, 877, 761, 626, 491, 377, 297, 222,
		219, 285, 370, 482, 618, 755, 883, 981, 1023, 996, 908, 785, 641, 503, 385, 300, 228,
		215, 281, 362, 474, 604, 741, 864, 960, 1001, 970, 887, 767, 631, 492, 377, 296, 223,
		206, 267, 343, 441, 562, 684, 795, 876, 910, 886, 814, 704, 580, 455, 356, 281, 212,
		192, 247, 315, 393, 494, 600, 692, 758, 785, 765, 706, 612, 506, 405, 324, 258, 199,
		173, 222, 279, 342, 416, 498, 572, 623, 645, 628, 580, 508, 426, 351, 286, 230, 176,
		152, 194, 240, 293, 345, 400, 452, 490, 502, 491, 455, 405, 352, 298, 245, 200, 158,
		128, 163, 200, 239, 279, 317, 346, 371, 377, 373, 349, 321, 285, 245, 204, 166, 133
	},
	{ /* Gr */
		142, 184, 220, 261, 301, 336, 371, 393, 401, 392, 371, 344, 306, 266, 222, 182, 143,
		171, 213, 264, 314, 369, 423, 472, 509, 521, 511, 478, 429, 374, 320, 266, 216, 173,
		190, 244, 302, 369, 440, 519, 590, 634, 652, 636, 592, 527, 449, 374, 309, 249, 193,
		210, 273, 340, 423, 520, 620, 701, 758, 780, 763, 710, 627, 529, 429, 347, 280, 214,
		226, 294, 373, 472, 587, 701, 801, 868, 898, 876, 811, 712, 598, 480, 379, 304, 231,
		238, 311, 394, 507, 636, 761, 874, 954, 985, 957, 883, 774, 645, 514, 401, 320, 240,
		243, 316, 405, 521, 656, 787, 905, 991, 1023, 994, 910, 796, 659, 525, 409, 322, 247,
		240, 312, 398, 514, 642, 772, 885, 970, 1003, 970, 891, 778, 649, 514, 401, 318, 240,
		230, 298, 378, 480, 600, 716, 820, 892, 919, 892, 823, 720, 601, 480, 381, 304, 231,
		216, 276, 347, 429, 531, 634, 721, 781, 803, 781, 723, 634, 530, 431, 349, 280, 217,
		196, 249, 309, 374, 451, 533, 605, 652, 670, 651, 605, 534, 453, 376, 311, 251, 193,
		172, 218, 267, 323, 378, 432, 485, 521, 532, 520, 483, 433, 378, 322, 267, 218, 172,
		145, 184, 223, 265, 308, 345, 374, 398, 404, 398, 374, 345, 308, 266, 223, 182, 145
	},
	{ /* Gb */
		144, 185, 222, 263, 305, 340, 376, 398, 407, 399, 378, 350, 311, 270, 226, 186, 146,
		174, 215, 266, 318, 374, 429, 479, 516, 530, 519, 487, 436, 380, 325, 271, 220, 176,
		193, 247, 305, 373, 444, 525, 597, 642, 662, 646, 602, 536, 457, 381, 314, 253, 196,
		213, 277, 343, 427, 523, 625, 707, 766, 789, 774, 721, 636, 537, 436, 353, 284, 217,
		230, 298, 375, 474, 588, 704, 804, 874, 905, 885, 820, 721, 605, 486, 384, 307, 233,
		241, 314, 396, 508, 635, 760, 873, 955, 989, 964, 891, 781, 652, 520, 406, 323, 242,
		246, 318, 406, 520, 652, 783, 900, 988, 1023, 998, 916, 802, 665, 530, 412, 324, 249,
		242, 314, 399, 512, 637, 767, 880, 966, 1001, 972, 895, 783, 654, 518, 405, 319, 241,
		232, 299, 378, 478, 596, 711, 814, 888, 917, 893, 827, 724, 605, 482, 384, 305, 232,
		217, 277, 348, 428, 528, 630, 718, 778, 802, 782, 727, 638, 534, 433, 351, 281, 218,
		197, 249, 308, 373, 449, 530, 602, 650, 670, 652, 608, 537, 456, 378, 312, 252, 194,
		173, 218, 267, 323, 377, 431, 484, 521, 532, 521, 486, 435, 380, 324, 269, 219, 173,
		146, 184, 223, 264, 307, 345, 373, 397, 404, 399, 376, 347, 310, 268, 225, 182, 146
	},
	{ /* B */
		152, 194, 230, 271, 313, 347, 381, 403, 410, 400, 377, 349, 312, 271, 225, 186, 146,
		182, 225, 276, 327, 383, 437, 485, 520, 532, 520, 485, 435, 380, 325, 271, 220, 177,
		201, 257, 316, 384, 454, 534, 604, 646, 662, 644, 598, 532, 455, 381, 315, 254, 198,
		221, 288, 356, 440, 536, 636, 715, 769, 788, 769, 715, 631, 534, 435, 354, 286, 219,
		239, 310, 390, 489, 603, 716, 814, 878, 904, 879, 812, 713, 600, 484, 385, 311, 237,
		251, 326, 412, 524, 651, 774, 884, 961, 989, 958, 882, 774, 647, 519, 407, 326, 245,
		257, 332, 422, 537, 669, 797, 911, 993, 1023, 991, 907, 794, 660, 529, 415, 329, 253,
		253, 328, 414, 529, 654, 780, 889, 970, 1000, 965, 886, 775, 649, 518, 407, 323, 245,
		242, 311, 393, 493, 610, 723, 822, 890, 914, 886, 818, 716, 601, 483, 387, 309, 235,
		224, 286, 361, 441, 541, 641, 724, 780, 799, 775, 719, 632, 531, 434, 354, 284, 220,
		203, 258, 319, 385, 460, 540, 610, 654, 669, 649, 604, 535, 455, 380, 315, 254, 196,
		179, 226, 276, 333, 387, 440, 491, 526, 535, 522, 484, 434, 380, 324, 270, 221, 175,
		152, 192, 231, 272, 315, 352, 380, 403, 408, 401, 376, 347, 309, 268, 226, 184, 148
	}
};


static int cam_eeprom_check_lsc(unsigned short int *lsc, int check_spec, int *sabun_max, int *sabun_min, int *sabun1_dat, int *sabun2_dat, int *sum_left, int *sum_right)
{
	int ret = -1;
	int sabun1[V_MAX - 2][H_MAX];
	int sabun2[V_MAX - 4][H_MAX];

	int row, col;
	int sum, sum_max, sum_min;

	unsigned short int *LSC[V_MAX];

	for (row = 0; row < V_MAX; row++)
	{
		LSC[row] = lsc + H_MAX * row;
	}

	for (row = 0; row < V_MAX - 2; row++)
	{
		for (col = 0; col < H_MAX; col++)
		{
			sabun1[row][col] = (LSC[row + 1][col] << 1) - LSC[row][col] - LSC[row + 2][col];
		}
	}

	for (row = 0; row < V_MAX - 4; row++)
	{
		for (col = 0; col < H_MAX; col++)
		{
			sabun2[row][col] = (sabun1[row + 1][col] << 1) - sabun1[row][col] - sabun1[row + 2][col];
		}
	}

	for (row = 0; row < V_MAX - 4; row++)
	{
		sum = 0;
		for (col = 0; col < 5; col++)
		{
			sum += sabun2[row][col];
		}

		sum_left[row] = sum;

		if (row == 0)
		{
			sum_max = sum;
			sum_min = sum;
		}
		else
		{
			sum_max = MAX(sum_max, sum);
			sum_min = MIN(sum_min, sum);
		}

		sum = 0;
		for (col = H_MAX - 5; col < H_MAX; col++)
		{
			sum += sabun2[row][col];
		}

		sum_right[row] = sum;

		sum_max = MAX(sum_max, sum);
		sum_min = MIN(sum_min, sum);
	}

	for (row = 0; row < V_MAX - 2; row++)
	{
		for (col = 0; col < H_MAX; col++)
		{
			sabun1_dat[row * H_MAX + col] = sabun1[row][col];
		}
	}
	for (row = 0; row < V_MAX - 4; row++)
	{
		for (col = 0; col < H_MAX; col++)
		{
			sabun2_dat[row * H_MAX + col] = sabun2[row][col];
		}
	}

	*sabun_max = sum_max;
	*sabun_min = sum_min;

	if (check_spec * -1 <= sum_min && sum_max <= check_spec) {
		ret = 0;
	}
	return ret;
}

static int cam_eeprom_test_lsc(unsigned short int *lsc)
{
	int check_spec = CHECK_SPEC;
	int check_result = -1;

	int sabun_max = 0;
	int sabun_min = 0;
	int *sabun1_dat = (int*)kzalloc((V_MAX - 2) * H_MAX * sizeof(int), GFP_KERNEL);
	int *sabun2_dat = (int*)kzalloc((V_MAX - 4) * H_MAX * sizeof(int), GFP_KERNEL);
	int *sum_right = (int*)kzalloc((V_MAX - 4) * sizeof(int), GFP_KERNEL);
	int *sum_left = (int*)kzalloc((V_MAX - 4) * sizeof(int), GFP_KERNEL);

	if(sabun1_dat != NULL && sabun2_dat != NULL && sum_right != NULL && sum_left != NULL) {
		// initialize
		memset(sabun1_dat, 0, (V_MAX - 2) * H_MAX * sizeof(int));
		memset(sabun2_dat, 0, (V_MAX - 4) * H_MAX * sizeof(int));
		memset(sum_right, 0, (V_MAX - 4) * sizeof(int));
		memset(sum_left, 0, (V_MAX - 4) * sizeof(int));

		// check lsc table
		check_result = cam_eeprom_check_lsc(lsc, check_spec, &sabun_max, &sabun_min, sabun1_dat, sabun2_dat, sum_left, sum_right);
	}

	if(sabun1_dat != NULL) {
		kfree(sabun1_dat);
	}
	if(sabun2_dat != NULL) {
		kfree(sabun2_dat);
	}
	if(sum_right != NULL) {
		kfree(sum_right);
	}
	if(sum_left != NULL) {
		kfree(sum_left);
	}

	return check_result;
}

static void cam_eeprom_convert_lsc_us(uint8_t *lsc_buf, uint16_t *lsc_us)
{
	uint32_t i = 0;
	for(i = 0; i < MESH_ROLLOFF_SIZE; i++) {
		lsc_us[i] = lsc_buf[i * 2] + (lsc_buf[i * 2 + 1] << 8);
	}
}
#endif /* Check LSC irregular value */

static void cam_eeprom_write_error_module_wb(uint8_t *wb_buf, uint16_t rg, uint16_t bg, uint16_t gg)
{
	wb_buf[0] = (uint8_t)((rg & 0x00FF));
	wb_buf[1] = (uint8_t)(rg >> 8);
	wb_buf[2] = (uint8_t)((bg & 0x00FF));
	wb_buf[3] = (uint8_t)(bg >> 8);
	wb_buf[4] = (uint8_t)((gg & 0x00FF));
	wb_buf[5] = (uint8_t)(gg >> 8);
}

static void cam_eeprom_write_error_module_lsc(uint8_t *lsc_buf, uint16_t *lsc)
{
	int32_t i;
	int32_t color;
	uint16_t val = 1023;
	uint8_t *buf;
	
	for(color = 0; color < 4; color++) {
		buf = lsc_buf + color * MESH_ROLLOFF_SIZE * 2;
		for(i = 0; i < MESH_ROLLOFF_SIZE; i++) {
			buf[i * 2] = lsc[color * MESH_ROLLOFF_SIZE + i] & 0x00FF;
			buf[i * 2 + 1] = (lsc[color * MESH_ROLLOFF_SIZE + i] & 0xFF00) >> 8;
		}
	}

	for(color = 0; color < 4; color++) {
		buf = lsc_buf + color * MESH_ROLLOFF_SIZE * 2;
		i = 18;
		buf[i * 2] = val & 0x00FF;
		buf[i * 2 + 1] = (val & 0xFF00) >> 8;
		i = 32;
		buf[i * 2] = val & 0x00FF;
		buf[i * 2 + 1] = (val & 0xFF00) >> 8;
		i = 188;
		buf[i * 2] = val & 0x00FF;
		buf[i * 2 + 1] = (val & 0xFF00) >> 8;
		i = 202;
		buf[i * 2] = val & 0x00FF;
		buf[i * 2 + 1] = (val & 0xFF00) >> 8;
	}
}
#endif /* reject OTP error module */

static void cam_eeprom_convert_grid(int32_t *src, int32_t *dst, int32_t src_w, int32_t src_h, int32_t dst_w, int32_t dst_h)
{
	int32_t src_x, src_y, dst_x, dst_y;
	int32_t ref_x_Q10, ref_y_Q10, dref_x_Q10, dref_y_Q10;

	for(dst_y = 0; dst_y < dst_h; dst_y++) {
		for(dst_x = 0; dst_x < dst_w; dst_x++) {
			ref_x_Q10 = 1024 * dst_x * (src_w - 1) / (dst_w - 1);
			ref_y_Q10 = 1024 * dst_y * (src_h - 1) / (dst_h - 1);
			src_x = (int32_t)(ref_x_Q10 / 1024);
			src_y = (int32_t)(ref_y_Q10 / 1024);
			dref_x_Q10 = ref_x_Q10 - src_x * 1024;
			dref_y_Q10 = ref_y_Q10 - src_y * 1024;

			dst[dst_w * dst_y + dst_x] = (1024 - dref_x_Q10) * (1024 - dref_y_Q10) * src[src_w * src_y + src_x]
				+ (1024 - dref_x_Q10) * dref_y_Q10 * src[src_w * MIN(src_y + 1, src_h - 1) + src_x]
				+ dref_x_Q10 * (1024 - dref_y_Q10) * src[src_w * src_y + MIN(src_x + 1, src_w - 1)]
				+ dref_x_Q10 * dref_y_Q10 * src[src_w * MIN(src_y + 1, src_h - 1) + MIN(src_x + 1, src_w - 1)];
			dst[dst_w * dst_y + dst_x] /= (1024 * 1024);
		}
	}

	return;
}

static void cam_eeprom_write_exlsc(int32_t *tmp_lsc_buf, uint8_t *otp_exlsc_buf)
{
	int32_t i;
	uint16_t val;
	for(i = 0; i < MESH_ROLLOFF_SIZE; i++) {
		val = (uint16_t)tmp_lsc_buf[i];
		otp_exlsc_buf[i * 2] = val & 0x00FF;
		otp_exlsc_buf[i * 2 + 1] = (val & 0xFF00) >> 8;
	}
}

static void cam_eeprom_read_imx351_lsc(uint8_t *otp_buf, int32_t *tmp_lsc_buf)
{
	int32_t i;
	for(i = 0; i < IMX351_OTP_LSC_SIZE; i++) {
		tmp_lsc_buf[i] = (uint32_t)((otp_buf[i * 2 + 1] << 8) + otp_buf[i * 2]);
	}
}

static void cam_eeprom_extract_imx351_lsc(uint8_t *otp_buf)
{
	int32_t *tmp_lsc_buf = (int32_t*)kzalloc(IMX351_OTP_LSC_SIZE * sizeof(int32_t), GFP_KERNEL);
	int32_t *tmp_exlsc_buf = (int32_t*)kzalloc(MESH_ROLLOFF_SIZE * sizeof(int32_t), GFP_KERNEL);
	uint8_t *otp_lsc_buf = otp_buf + IMX351_OTP_LSC_OFFSET;
	uint8_t *otp_exlsc_buf = otp_buf + IMX351_OTP_EXLSC_OFFSET;

	if(tmp_lsc_buf != NULL && tmp_exlsc_buf != NULL) {
		cam_eeprom_read_imx351_lsc(otp_lsc_buf, tmp_lsc_buf);
		cam_eeprom_convert_grid(tmp_lsc_buf, tmp_exlsc_buf, IMX351_OTP_LSC_W, IMX351_OTP_LSC_H, MESH_ROLLOFF_W, MESH_ROLLOFF_H);
		cam_eeprom_write_exlsc(tmp_exlsc_buf, otp_exlsc_buf);

		otp_lsc_buf += IMX351_OTP_LSC_SIZE * 2;
		otp_exlsc_buf += MESH_ROLLOFF_SIZE * 2;
		cam_eeprom_read_imx351_lsc(otp_lsc_buf, tmp_lsc_buf);
		cam_eeprom_convert_grid(tmp_lsc_buf, tmp_exlsc_buf, IMX351_OTP_LSC_W, IMX351_OTP_LSC_H, MESH_ROLLOFF_W, MESH_ROLLOFF_H);
		cam_eeprom_write_exlsc(tmp_exlsc_buf, otp_exlsc_buf);

		otp_lsc_buf += IMX351_OTP_LSC_SIZE * 2;
		otp_exlsc_buf += MESH_ROLLOFF_SIZE * 2;
		cam_eeprom_read_imx351_lsc(otp_lsc_buf, tmp_lsc_buf);
		cam_eeprom_convert_grid(tmp_lsc_buf, tmp_exlsc_buf, IMX351_OTP_LSC_W, IMX351_OTP_LSC_H, MESH_ROLLOFF_W, MESH_ROLLOFF_H);
		cam_eeprom_write_exlsc(tmp_exlsc_buf, otp_exlsc_buf);

		otp_lsc_buf += IMX351_OTP_LSC_SIZE * 2;
		otp_exlsc_buf += MESH_ROLLOFF_SIZE * 2;
		cam_eeprom_read_imx351_lsc(otp_lsc_buf, tmp_lsc_buf);
		cam_eeprom_convert_grid(tmp_lsc_buf, tmp_exlsc_buf, IMX351_OTP_LSC_W, IMX351_OTP_LSC_H, MESH_ROLLOFF_W, MESH_ROLLOFF_H);
		cam_eeprom_write_exlsc(tmp_exlsc_buf, otp_exlsc_buf);
	}
	
	if(tmp_lsc_buf != NULL) {
		kfree(tmp_lsc_buf);
	}
	if(tmp_exlsc_buf != NULL) {
		kfree(tmp_exlsc_buf);
	}
}

static void cam_eeprom_swap_data(uint8_t *buf1, uint8_t *buf2, uint32_t swap_size)
{
	uint32_t i = 0;
	uint8_t tmp;
	for(i = 0; i < swap_size; i++) {
		tmp = *(buf1 + i);
		*(buf1 + i) = *(buf2 + i);
		*(buf2 + i) = tmp;
	}
}

static void cam_eeprom_rotate_exlsc(uint8_t *otp_buf, uint32_t exlsc_offset)
{
	uint32_t i = 0;
	uint8_t *otp_exlsc_buf = otp_buf + exlsc_offset;

	for(i = 0; i < MESH_ROLLOFF_SIZE / 2; i++) {
		cam_eeprom_swap_data(otp_exlsc_buf + i * 2, otp_exlsc_buf + (MESH_ROLLOFF_SIZE - 1 - i) * 2, 2);
	}

	otp_exlsc_buf += MESH_ROLLOFF_SIZE * 2;
	for(i = 0; i < MESH_ROLLOFF_SIZE / 2; i++) {
		cam_eeprom_swap_data(otp_exlsc_buf + i * 2, otp_exlsc_buf + (MESH_ROLLOFF_SIZE - 1 - i) * 2, 2);
	}

	otp_exlsc_buf += MESH_ROLLOFF_SIZE * 2;
	for(i = 0; i < MESH_ROLLOFF_SIZE / 2; i++) {
		cam_eeprom_swap_data(otp_exlsc_buf + i * 2, otp_exlsc_buf + (MESH_ROLLOFF_SIZE - 1 - i) * 2, 2);
	}

	otp_exlsc_buf += MESH_ROLLOFF_SIZE * 2;
	for(i = 0; i < MESH_ROLLOFF_SIZE / 2; i++) {
		cam_eeprom_swap_data(otp_exlsc_buf + i * 2, otp_exlsc_buf + (MESH_ROLLOFF_SIZE - 1 - i) * 2, 2);
	}
}
/* SHLOCAL_CAMERA_IMAGE_QUALITY<- */

/**
 * cam_eeprom_get_cal_data - parse the userspace IO config and
 *                                        copy read data to share with userspace
 * @e_ctrl:     ctrl structure
 * @csl_packet: csl packet received
 *
 * Returns success or failure
 */
static int32_t cam_eeprom_get_cal_data(struct cam_eeprom_ctrl_t *e_ctrl,
	struct cam_packet *csl_packet)
{
	struct cam_buf_io_cfg *io_cfg;
	uint32_t              i = 0;
	int                   rc = 0;
	uintptr_t              buf_addr;
	size_t                buf_size;
	uint8_t               *read_buffer;
	size_t                remain_len = 0;
/* SHLOCAL_CAMERA_IMAGE_QUALITY-> *//* reject OTP error module */
#if 0
	int otp_err = 0;
	uint32_t otp_wb_offset = 0;
	uint32_t otp_lsc_offset = 0;
	uint16_t otp_wb_dummy_rg = 614;
	uint16_t otp_wb_dummy_bg = 614;
	uint16_t otp_wb_dummy_gg = 1024;
#endif
/* SHLOCAL_CAMERA_IMAGE_QUALITY<- */

	io_cfg = (struct cam_buf_io_cfg *) ((uint8_t *)
		&csl_packet->payload +
		csl_packet->io_configs_offset);

	CAM_DBG(CAM_EEPROM, "number of IO configs: %d:",
		csl_packet->num_io_configs);

	for (i = 0; i < csl_packet->num_io_configs; i++) {
		CAM_DBG(CAM_EEPROM, "Direction: %d:", io_cfg->direction);
		if (io_cfg->direction == CAM_BUF_OUTPUT) {
			rc = cam_mem_get_cpu_buf(io_cfg->mem_handle[0],
				&buf_addr, &buf_size);
			if (rc) {
				CAM_ERR(CAM_EEPROM, "Fail in get buffer: %d",
					rc);
				return rc;
			}
			if (buf_size <= io_cfg->offsets[0]) {
				CAM_ERR(CAM_EEPROM, "Not enough buffer");
				rc = -EINVAL;
				goto rel_cmd_buf;
			}

			remain_len = buf_size - io_cfg->offsets[0];
			CAM_DBG(CAM_EEPROM, "buf_addr : %pK, buf_size : %zu\n",
				(void *)buf_addr, buf_size);

			read_buffer = (uint8_t *)buf_addr;
			if (!read_buffer) {
				CAM_ERR(CAM_EEPROM,
					"invalid buffer to copy data");
				rc = -EINVAL;
				goto rel_cmd_buf;
			}
			read_buffer += io_cfg->offsets[0];

			if (remain_len < e_ctrl->cal_data.num_data) {
				CAM_ERR(CAM_EEPROM,
					"failed to copy, Invalid size");
				rc = -EINVAL;
				goto rel_cmd_buf;
			}

			CAM_DBG(CAM_EEPROM, "copy the data, len:%d",
				e_ctrl->cal_data.num_data);
			memcpy(read_buffer, e_ctrl->cal_data.mapdata,
					e_ctrl->cal_data.num_data);
			if (cam_mem_put_cpu_buf(io_cfg->mem_handle[0]))
				CAM_WARN(CAM_EEPROM, "Fail in put buffer: 0x%x",
					io_cfg->mem_handle[0]);

/* SHLOCAL_CAMERA_IMAGE_QUALITY-> *//* lsc for imx563/imx351/imx476 */
			/*
			  use buf_size to identify camera module. buf_size derives from registerData of \chi-cdk\vendor\eeprom\xxx_eeprom.xml.
			  imx563  : buf_size = 4840
			  imx476  : buf_size = 5592
			  imx351  : buf_size = 2344
			*/
			
			/* imx351 */
			if(buf_size == 2344) {
				/* extend eeprom area for imx351 lsc */
				CAM_DBG(CAM_EEPROM, "extend eeprom area for imx351 lsc. buf_size=%zu num_data=%d", buf_size, e_ctrl->cal_data.num_data);
				cam_eeprom_extract_imx351_lsc(read_buffer);
			}

			/* imx476 */
			if(buf_size == 5592) {
				/* rotate lsc table */
				cam_eeprom_rotate_exlsc(read_buffer, IMX476_OTP_LSC_OFFSET);
			}

			/* imx351 */
			if(buf_size == 2344) {
				/* rotate lsc table */
				cam_eeprom_rotate_exlsc(read_buffer, IMX351_OTP_EXLSC_OFFSET);
			}
/* SHLOCAL_CAMERA_IMAGE_QUALITY<- */

/* SHLOCAL_CAMERA_IMAGE_QUALITY-> *//* reject OTP error module */
#if 0
			/* imx563 */
			if(buf_size == 4840) {
				otp_wb_offset = IMX563_OTP_WB_OFFSET;
				otp_lsc_offset = IMX563_OTP_LSC_OFFSET;
				otp_wb_dummy_rg = 609;  /* random module */
				otp_wb_dummy_bg = 563;  /* random module */
				otp_wb_dummy_gg = 1024; /* random module */
				if((read_buffer[IMX563_OTP_WB_OFFSET] == 0xFF && read_buffer[IMX563_OTP_WB_OFFSET+1] == 0xFF) || /* R/Gr */
				  (read_buffer[IMX563_OTP_VCMOIS_OFFSET] == 0xFF && read_buffer[IMX563_OTP_VCMOIS_OFFSET+1] == 0xFF) || /* CURDAT */
				  (read_buffer[IMX563_OTP_VCMAF_OFFSET] == 0xFF && read_buffer[IMX563_OTP_VCMAF_OFFSET+1] == 0xFF) || /* Code_10cm */
				  (read_buffer[IMX563_OTP_LSC_OFFSET] == 0xFF && read_buffer[IMX563_OTP_LSC_OFFSET+1] == 0xFF) || /* Area000R */
				  (read_buffer[IMX563_OTP_LRC_OFFSET+2] == 0xFF && read_buffer[IMX563_OTP_LRC_OFFSET+3] == 0xFF) || /* VersionNumber */
				  (read_buffer[IMX563_OTP_DCC_OFFSET+2] == 0xFF)) /* DCC Q Format H */
				{
					otp_err = 1;
					CAM_ERR(CAM_EEPROM, "OTP/EEPROM data is invalid !");
				}
				else {
#if 1 /* Check LSC irregular value */
					uint16_t *lsc_r = (int16_t*)kzalloc(MESH_ROLLOFF_SIZE * sizeof(int16_t), GFP_KERNEL);
					cam_eeprom_convert_lsc_us(read_buffer + otp_lsc_offset, lsc_r);
					if(cam_eeprom_test_lsc(lsc_r) == -1) { /* r only */
						otp_err = 1;
					}
					else {
						otp_err = 0;
					}
					if(lsc_r != NULL) {
						kfree(lsc_r);
					}
#endif
				}
			}

			/* imx476 */
			if(buf_size == 5592) {
				otp_wb_offset = IMX476_OTP_WB_OFFSET;
				otp_lsc_offset = IMX476_OTP_LSC_OFFSET;
				otp_wb_dummy_rg = 520;  /* random module */
				otp_wb_dummy_bg = 508;  /* random module */
				otp_wb_dummy_gg = 1024; /* random module */
				if((read_buffer[IMX476_OTP_WB_OFFSET] == 0xFF && (read_buffer[IMX476_OTP_WB_OFFSET+1] & 0x07) == 0x07) || /* R/Gr */
					(read_buffer[IMX476_OTP_LSC_OFFSET] == 0xFF && (read_buffer[IMX476_OTP_LSC_OFFSET+1] & 0x03) == 0x03) || /* LSC_R_00_00 */
					(read_buffer[IMX476_OTP_QSC_OFFSET] == 0xFF && read_buffer[IMX476_OTP_QSC_OFFSET+1] == 0xFF && read_buffer[IMX476_OTP_QSC_OFFSET+2] == 0xFF) || /* QSC_KNOT_POINT_R_00_0 */
					(read_buffer[IMX476_OTP_DPC_OFFSET+2030] == 0xFF && read_buffer[IMX476_OTP_DPC_OFFSET+2031] == 0xFF) || /* byte_count */
					((read_buffer[IMX476_OTP_LSC_QSC_OFFSET] & 0x03) == 0x03 && read_buffer[IMX476_OTP_LSC_QSC_OFFSET+1] == 0xFF)) /* LSC_TABLE_R_00 */
				{
					otp_err = 1;
					CAM_ERR(CAM_EEPROM, "OTP/EEPROM data is invalid !");
				}
				else {
					otp_err = 0;
				}
			}

			/* imx351 */
			if(buf_size == 2344) {
				otp_wb_offset = IMX351_OTP_WB_OFFSET;
				otp_lsc_offset = IMX351_OTP_EXLSC_OFFSET;
				otp_wb_dummy_rg = 592;  /* random module */
				otp_wb_dummy_bg = 606;  /* random module */
				otp_wb_dummy_gg = 1024; /* random module */
				if((read_buffer[IMX351_OTP_WB_OFFSET] == 0x00 && read_buffer[IMX351_OTP_WB_OFFSET+1] == 0x00) || /* R/Gr */
					(read_buffer[IMX351_OTP_LSC_OFFSET] == 0x00 && read_buffer[IMX351_OTP_LSC_OFFSET+1] == 0x00)) /* Area000R */
				{
					otp_err = 1;
					CAM_ERR(CAM_EEPROM, "OTP/EEPROM data is invalid !");
				}
				else {
					otp_err = 0;
				}
			}

			if(otp_err == 1) {
				cam_eeprom_write_error_module_wb(read_buffer + otp_wb_offset, otp_wb_dummy_rg, otp_wb_dummy_bg, otp_wb_dummy_gg);
				cam_eeprom_write_error_module_lsc(read_buffer + otp_lsc_offset, &otp_lsc_dummy[0][0]);
			}
#endif
/* SHLOCAL_CAMERA_IMAGE_QUALITY<- */

		} else {
			CAM_ERR(CAM_EEPROM, "Invalid direction");
			rc = -EINVAL;
		}
	}

	return rc;

rel_cmd_buf:
	if (cam_mem_put_cpu_buf(io_cfg->mem_handle[0]))
		CAM_WARN(CAM_EEPROM, "Fail in put buffer : 0x%x",
			io_cfg->mem_handle[0]);

	return rc;
}

/**
 * cam_eeprom_pkt_parse - Parse csl packet
 * @e_ctrl:     ctrl structure
 * @arg:        Camera control command argument
 *
 * Returns success or failure
 */
static int32_t cam_eeprom_pkt_parse(struct cam_eeprom_ctrl_t *e_ctrl, void *arg)
{
	int32_t                         rc = 0;
	struct cam_control             *ioctl_ctrl = NULL;
	struct cam_config_dev_cmd       dev_config;
	uintptr_t                        generic_pkt_addr;
	size_t                          pkt_len;
	size_t                          remain_len = 0;
	struct cam_packet              *csl_packet = NULL;
	struct cam_eeprom_soc_private  *soc_private =
		(struct cam_eeprom_soc_private *)e_ctrl->soc_info.soc_private;
	struct cam_sensor_power_ctrl_t *power_info = &soc_private->power_info;

	ioctl_ctrl = (struct cam_control *)arg;

	if (copy_from_user(&dev_config,
		u64_to_user_ptr(ioctl_ctrl->handle),
		sizeof(dev_config)))
		return -EFAULT;
	rc = cam_mem_get_cpu_buf(dev_config.packet_handle,
		&generic_pkt_addr, &pkt_len);
	if (rc) {
		CAM_ERR(CAM_EEPROM,
			"error in converting command Handle Error: %d", rc);
		return rc;
	}

	remain_len = pkt_len;
	if ((sizeof(struct cam_packet) > pkt_len) ||
		((size_t)dev_config.offset >= pkt_len -
		sizeof(struct cam_packet))) {
		CAM_ERR(CAM_EEPROM,
			"Inval cam_packet strut size: %zu, len_of_buff: %zu",
			 sizeof(struct cam_packet), pkt_len);
		rc = -EINVAL;
		goto release_buf;
	}

	remain_len -= (size_t)dev_config.offset;
	csl_packet = (struct cam_packet *)
		(generic_pkt_addr + (uint32_t)dev_config.offset);

	if (cam_packet_util_validate_packet(csl_packet,
		remain_len)) {
		CAM_ERR(CAM_EEPROM, "Invalid packet params");
		rc = -EINVAL;
		goto release_buf;
	}

	switch (csl_packet->header.op_code & 0xFFFFFF) {
	case CAM_EEPROM_PACKET_OPCODE_INIT:
		if (e_ctrl->userspace_probe == false) {
			rc = cam_eeprom_parse_read_memory_map(
					e_ctrl->soc_info.dev->of_node, e_ctrl);
			if (rc < 0) {
				CAM_ERR(CAM_EEPROM, "Failed: rc : %d", rc);
				goto release_buf;
			}
			rc = cam_eeprom_get_cal_data(e_ctrl, csl_packet);
			vfree(e_ctrl->cal_data.mapdata);
			vfree(e_ctrl->cal_data.map);
			e_ctrl->cal_data.num_data = 0;
			e_ctrl->cal_data.num_map = 0;
			CAM_DBG(CAM_EEPROM,
				"Returning the data using kernel probe");
			break;
		}
		rc = cam_eeprom_init_pkt_parser(e_ctrl, csl_packet);
		if (rc) {
			CAM_ERR(CAM_EEPROM,
				"Failed in parsing the pkt");
			goto release_buf;
		}

		e_ctrl->cal_data.mapdata =
			vzalloc(e_ctrl->cal_data.num_data);
		if (!e_ctrl->cal_data.mapdata) {
			rc = -ENOMEM;
			CAM_ERR(CAM_EEPROM, "failed");
			goto error;
		}

		rc = cam_eeprom_power_up(e_ctrl,
			&soc_private->power_info);
		if (rc) {
			CAM_ERR(CAM_EEPROM, "failed rc %d", rc);
			goto memdata_free;
		}

		e_ctrl->cam_eeprom_state = CAM_EEPROM_CONFIG;
		rc = cam_eeprom_read_memory(e_ctrl, &e_ctrl->cal_data);
		if (rc) {
			CAM_ERR(CAM_EEPROM,
				"read_eeprom_memory failed");
			goto power_down;
		}

		rc = cam_eeprom_get_cal_data(e_ctrl, csl_packet);
		rc = cam_eeprom_power_down(e_ctrl);
		e_ctrl->cam_eeprom_state = CAM_EEPROM_ACQUIRE;
		vfree(e_ctrl->cal_data.mapdata);
		vfree(e_ctrl->cal_data.map);
		kfree(power_info->power_setting);
		kfree(power_info->power_down_setting);
		power_info->power_setting = NULL;
		power_info->power_down_setting = NULL;
		power_info->power_setting_size = 0;
		power_info->power_down_setting_size = 0;
		e_ctrl->cal_data.num_data = 0;
		e_ctrl->cal_data.num_map = 0;
		break;
	default:
		break;
	}

	if (cam_mem_put_cpu_buf(dev_config.packet_handle))
		CAM_WARN(CAM_EEPROM, "Put cpu buffer failed : 0x%x",
			dev_config.packet_handle);

	return rc;

power_down:
	cam_eeprom_power_down(e_ctrl);
memdata_free:
	vfree(e_ctrl->cal_data.mapdata);
error:
	kfree(power_info->power_setting);
	kfree(power_info->power_down_setting);
	power_info->power_setting = NULL;
	power_info->power_down_setting = NULL;
	vfree(e_ctrl->cal_data.map);
	e_ctrl->cal_data.num_data = 0;
	e_ctrl->cal_data.num_map = 0;
	e_ctrl->cam_eeprom_state = CAM_EEPROM_INIT;
release_buf:
	if (cam_mem_put_cpu_buf(dev_config.packet_handle))
		CAM_WARN(CAM_EEPROM, "Put cpu buffer failed : 0x%x",
			dev_config.packet_handle);

	return rc;
}

void cam_eeprom_shutdown(struct cam_eeprom_ctrl_t *e_ctrl)
{
	int rc;
	struct cam_eeprom_soc_private  *soc_private =
		(struct cam_eeprom_soc_private *)e_ctrl->soc_info.soc_private;
	struct cam_sensor_power_ctrl_t *power_info = &soc_private->power_info;

	if (e_ctrl->cam_eeprom_state == CAM_EEPROM_INIT)
		return;

	if (e_ctrl->cam_eeprom_state == CAM_EEPROM_CONFIG) {
		rc = cam_eeprom_power_down(e_ctrl);
		if (rc < 0)
			CAM_ERR(CAM_EEPROM, "EEPROM Power down failed");
		e_ctrl->cam_eeprom_state = CAM_EEPROM_ACQUIRE;
	}

	if (e_ctrl->cam_eeprom_state == CAM_EEPROM_ACQUIRE) {
		rc = cam_destroy_device_hdl(e_ctrl->bridge_intf.device_hdl);
		if (rc < 0)
			CAM_ERR(CAM_EEPROM, "destroying the device hdl");

		e_ctrl->bridge_intf.device_hdl = -1;
		e_ctrl->bridge_intf.link_hdl = -1;
		e_ctrl->bridge_intf.session_hdl = -1;

		kfree(power_info->power_setting);
		kfree(power_info->power_down_setting);
		power_info->power_setting = NULL;
		power_info->power_down_setting = NULL;
		power_info->power_setting_size = 0;
		power_info->power_down_setting_size = 0;
	}

	e_ctrl->cam_eeprom_state = CAM_EEPROM_INIT;
}

/**
 * cam_eeprom_driver_cmd - Handle eeprom cmds
 * @e_ctrl:     ctrl structure
 * @arg:        Camera control command argument
 *
 * Returns success or failure
 */
int32_t cam_eeprom_driver_cmd(struct cam_eeprom_ctrl_t *e_ctrl, void *arg)
{
	int                            rc = 0;
	struct cam_eeprom_query_cap_t  eeprom_cap = {0};
	struct cam_control            *cmd = (struct cam_control *)arg;

	if (!e_ctrl || !cmd) {
		CAM_ERR(CAM_EEPROM, "Invalid Arguments");
		return -EINVAL;
	}

	if (cmd->handle_type != CAM_HANDLE_USER_POINTER) {
		CAM_ERR(CAM_EEPROM, "Invalid handle type: %d",
			cmd->handle_type);
		return -EINVAL;
	}

	mutex_lock(&(e_ctrl->eeprom_mutex));
	switch (cmd->op_code) {
	case CAM_QUERY_CAP:
		eeprom_cap.slot_info = e_ctrl->soc_info.index;
		if (e_ctrl->userspace_probe == false)
			eeprom_cap.eeprom_kernel_probe = true;
		else
			eeprom_cap.eeprom_kernel_probe = false;

		if (copy_to_user(u64_to_user_ptr(cmd->handle),
			&eeprom_cap,
			sizeof(struct cam_eeprom_query_cap_t))) {
			CAM_ERR(CAM_EEPROM, "Failed Copy to User");
			return -EFAULT;
			goto release_mutex;
		}
		CAM_DBG(CAM_EEPROM, "eeprom_cap: ID: %d", eeprom_cap.slot_info);
		break;
	case CAM_ACQUIRE_DEV:
		rc = cam_eeprom_get_dev_handle(e_ctrl, arg);
		if (rc) {
			CAM_ERR(CAM_EEPROM, "Failed to acquire dev");
			goto release_mutex;
		}
		e_ctrl->cam_eeprom_state = CAM_EEPROM_ACQUIRE;
		break;
	case CAM_RELEASE_DEV:
		if (e_ctrl->cam_eeprom_state != CAM_EEPROM_ACQUIRE) {
			rc = -EINVAL;
			CAM_WARN(CAM_EEPROM,
			"Not in right state to release : %d",
			e_ctrl->cam_eeprom_state);
			goto release_mutex;
		}

		if (e_ctrl->bridge_intf.device_hdl == -1) {
			CAM_ERR(CAM_EEPROM,
				"Invalid Handles: link hdl: %d device hdl: %d",
				e_ctrl->bridge_intf.device_hdl,
				e_ctrl->bridge_intf.link_hdl);
			rc = -EINVAL;
			goto release_mutex;
		}
		rc = cam_destroy_device_hdl(e_ctrl->bridge_intf.device_hdl);
		if (rc < 0)
			CAM_ERR(CAM_EEPROM,
				"failed in destroying the device hdl");
		e_ctrl->bridge_intf.device_hdl = -1;
		e_ctrl->bridge_intf.link_hdl = -1;
		e_ctrl->bridge_intf.session_hdl = -1;
		e_ctrl->cam_eeprom_state = CAM_EEPROM_INIT;
		break;
	case CAM_CONFIG_DEV:
		rc = cam_eeprom_pkt_parse(e_ctrl, arg);
		if (rc) {
			CAM_ERR(CAM_EEPROM, "Failed in eeprom pkt Parsing");
			goto release_mutex;
		}
		break;
	default:
		CAM_DBG(CAM_EEPROM, "invalid opcode");
		break;
	}

release_mutex:
	mutex_unlock(&(e_ctrl->eeprom_mutex));

	return rc;
}

