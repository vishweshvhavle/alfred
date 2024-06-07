# Alfred UGV

This is the official implementation of [Alfred UGV](https://instruct-nerf2nerf.github.io/).

![teaser](imgs/in2n_teaser.png)

# Installation

## 1. Install Nerfstudio dependencies

Instruct-NeRF2NeRF is build on Nerfstudio and therefore has the same dependency reqirements. Specfically [PyTorch](https://pytorch.org/) and [tinycudann](https://github.com/NVlabs/tiny-cuda-nn) are required.

Follow the instructions [at this link](https://docs.nerf.studio/quickstart/installation.html) to create the environment and install dependencies. Only follow the commands up to tinycudann. After the dependencies have been installed, return here.

## 2. Installing Instruct-NeRF2NeRF

Once you have finished installing dependencies, you can install Instruct-NeRF2NeRF using the following command:
```bash
pip install git+https://github.com/ayaanzhaque/instruct-nerf2nerf
```

_Optional_: If you would like to work with the code directly, clone then install the repo:
```bash
git clone https://github.com/ayaanzhaque/instruct-nerf2nerf.git
cd instruct-nerf2nerf
pip install --upgrade pip setuptools
pip install -e .
```

## 3. Checking the install

The following command should include `in2n` as one of the options:
```bash
ns-train -h
```

# Using Instruct-NeRF2NeRF

![teaser](imgs/in2n_pipeline.png)

To edit a NeRF, you must first train a regular `nerfacto` scene using your data. To process your custom data, please refer to [this](https://docs.nerf.studio/quickstart/custom_dataset.html) documentation.

Once you have your custom data, you can train your initial NeRF with the following command:

```bash
ns-train nerfacto --data {PROCESSED_DATA_DIR}
```

For more details on training a NeRF, see [Nerfstudio documentation](https://docs.nerf.studio/quickstart/first_nerf.html).

Once you have fully trained your scene, the checkpoints will be saved to the `outputs` directory. Copy the path to the `nerfstudio_models` folder.

To start training for editing the NeRF, run the following command:

```bash
ns-train in2n --data {PROCESSED_DATA_DIR} --load-dir {outputs/.../nerfstudio_models} --pipeline.prompt {"prompt"} --pipeline.guidance-scale 7.5 --pipeline.image-guidance-scale 1.5
```

The `{PROCESSED_DATA_DIR}` must be the same path as used in training the original NeRF. Using the CLI commands, you can choose the prompt and the guidance scales used for InstructPix2Pix.

After the NeRF is trained, you can render the NeRF using the standard Nerfstudio workflow, found [here](https://docs.nerf.studio/quickstart/viewer_quickstart.html).

## Training Notes

***Important***
Please note that training the NeRF on images with resolution larger than 512 will likely cause InstructPix2Pix to throw OOM errors. Moreover, it seems InstructPix2Pix performs significantly worse on images at higher resolution. We suggest training with a resolution that is around 512 (max dimension), so add the following tag to the end of both your `nerfacto` and `in2n` training command: `nerfstudio-data --downscale-factor {2,4,6,8}` to the end of your `ns-train` commands. Alternatively, you can downscale your dataset yourself and update your `transforms.json` file (scale down w, h, fl_x, fl_y, cx, cy), or you can use a smaller image scale provided by Nerfstudio.

We recommend capturing data using images from Polycam, as smaller datasets work better and faster with our method.

If you have multiple GPUs, training can be sped up by placing InstructPix2Pix on a separate GPU. To do so, add `--pipeline.ip2p-device cuda:{device-number}` to your training command.

Our method uses ~16K rays and LPIPS, but not all GPUs have enough memory to run this configuration. As a result, we have provided two alternative configurations which use less memory, but be aware that these configurations lead to decreased performance. The differences are the precision used for IntructPix2Pix and whether LPIPS is used (which requires 4x more rays). The details of each config is provided in the table below.

| Method | Description | Memory | Quality |
| ---------------------------------------------------------------------------------------------------- | -------------- | ----------------------------------------------------------------- | ----------------------- |
| `in2n` | Full model, used in paper | ~15GB | Best |
| `in2n-small` | Half precision model | ~12GB | Good |
| `in2n-tiny` | Half precision with no LPIPS | ~10GB | Ok |

Currently, we set the max number of iterations for `in2n` training to be 15k iteratios. Most often, the edit will look good after ~10k iterations. If you would like to train for longer, just reload your last `in2n` checkpoint and continue training, or change `--max-num-iterations 30000`.

## Tips

If your edit isn't working as you desire, it is likely because InstructPix2Pix struggles with your images and prompt. We recommend taking one of your training views and trying to edit it in 2D first with InstructPix2Pix, which can be done at [this](https://huggingface.co/spaces/timbrooks/instruct-pix2pix) HuggingFace space. More tips on getting a good edit can be found [here](https://github.com/timothybrooks/instruct-pix2pix#tips).

# Extending Instruct-NeRF2NeRF

We built an extension of Instruct-NeRF2NeRF for Gaussian Splatting called [Instruct-GS2GS](https://instruct-gs2gs.github.io/). Their [repository](https://github.com/cvachha/instruct-gs2gs) can be used as an example of how to build future projects based on Instruct-NeRF2NeRF.

### Issues
Please open Github issues for any installation/usage problems you run into. We've tried to support as broad a range of GPUs as possible, but it might be necessary to provide even more low-footprint versions. Please contribute with any changes to improve memory usage!

### Code structure
To build off Instruct-NeRF2NeRF, we provide explanations of the core code components.

`in2n_datamanager.py`: This file is almost identical to the `base_datamanager.py` in Nerfstudio. The main difference is that the entire dataset tensor is pre-computed in the `setup_train` method as opposed to being sampled in the `next_train` method each time.

`in2n_pipeline.py`: This file builds on the pipeline module in Nerfstudio. The `get_train_loss_dict` method samples images and places edited images back into the dataset.

`ip2p.py`: This file houses the InstructPix2Pix model (using the `diffusers` implementation). The `edit_image` method is where an image is denoised using the diffusion model, and a variety of helper methods are contained in this file as well.

`in2n.py`: We overwrite the `get_loss_dict` method to use LPIPs loss and L1Loss.

# Citation

If you find this code useful for your research, please consider citing:

```
@inproceedings{alfred2024,
    author = {Vishwesh and Jatin},
    title = {Alfred UGV},
    year = {2024},
}
```
