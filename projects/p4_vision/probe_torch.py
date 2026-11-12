import torch, torchvision
print("torch", torch.__version__, "tv", torchvision.__version__, "cuda", torch.cuda.is_available(),
      torch.cuda.get_device_name(0) if torch.cuda.is_available() else "")
