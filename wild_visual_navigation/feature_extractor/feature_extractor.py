from wild_visual_navigation.feature_extractor import StegoInterface
from wild_visual_navigation.utils import PlotHelper

import torch.nn.functional as F
import skimage
import torch
import numpy as np
import kornia
import cv2
from torchvision import transforms as T
from PIL import Image, ImageDraw


class SegmentExtractor(torch.nn.Module):
    @torch.no_grad()
    def __init__(self):
        super().__init__()
        # Use Convolutional Filter to Extract Edges
        self.f1 = torch.nn.Conv2d(1, 4, (3, 3), padding_mode="replicate", padding=(3, 3), bias=False)
        self.f1.weight[:, :, :, :] = 0
        # 0  0  0
        # 0  1 -1
        # 0  0  0
        self.f1.weight[0, 0, 1, 1] = 1
        self.f1.weight[0, 0, 1, 2] = -1
        # 0  0  0
        # 1 -1  0
        # 0  0  0
        self.f1.weight[1, 0, 1, 0] = 1
        self.f1.weight[1, 0, 1, 1] = -1
        # 0  0  0
        # 0  1  0
        # 0 -1  0
        self.f1.weight[2, 0, 1, 1] = 1
        self.f1.weight[2, 0, 2, 1] = -1
        # 0  0  0
        # 0 -1  0
        # 0  1  0
        self.f1.weight[3, 0, 0, 1] = 1
        self.f1.weight[3, 0, 1, 1] = -1

    @torch.no_grad()
    def adjacency_list(self, seg):
        """Extracts a adjacency list based on neigbooring classes.

        Args:
            seg (torch.Tensor, dtype=torch.long, shape=(BS, 1, H, W)): Segmentation

        Returns:
            adjacency_list (torch.Tensor, dtype=torch.long, shape=(BS, N, 2): Adjacency list of undirected graph
        """
        assert seg.shape[0] == 1 and len(seg.shape) == 4

        res = self.f1(seg.type(torch.float32))
        boundary_mask = (res != 0)[0, :, 2:-2, 2:-2]

        # Shifting the filter allows to index the left and right segment of a bordered
        left_idx = torch.cat([seg[0, 0, boundary_mask[0]], seg[0, 0, boundary_mask[2]]])
        right_idx = torch.cat([seg[0, 0, boundary_mask[1]], seg[0, 0, boundary_mask[3]]])

        # Create adjeceney_list based on the given pairs (crucial to use float64 here)
        div = seg.max() + 1
        unique_key = (left_idx + (right_idx * (div))).type(torch.float64)
        m = torch.unique(unique_key)

        le_idx = (m % div).type(torch.long)
        ri_idx = torch.floor(m / div).type(torch.long)
        adjacency_list = torch.stack([le_idx, ri_idx], dim=1)

        return adjacency_list[None]

    @torch.no_grad()
    def centers(self, seg):
        """Extracts a center position in image plane of clusters.

        Args:
            seg (torch.Tensor, dtype=torch.long, shape=(BS, 1, H, W)): Segmentation

        Returns:
            centers (torch.Tensor, dtype=torch.long, shape=(BS, N, 2): Center position in image plane of clusters.
        """

        assert seg.shape[0] == 1 and len(seg.shape) == 4

        centers = []
        tmp_seg = seg.T
        for s in range(seg.max() + 1):
            indices = torch.nonzero((s == tmp_seg)[:, :, 0, 0])
            res = indices.type(torch.float32).mean(dim=0)
            centers.append(res)
        centers = torch.stack(centers)

        return centers[None]


class FeatureExtractor:
    def __init__(self, device):
        """Feature extraction from image

        Args:
            device (str): Compute device
        """

        self.device = device
        self.si = StegoInterface(device=device)

        self.crop = T.Compose([T.Resize(448, Image.NEAREST), T.CenterCrop(448)])
        self.norm = T.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225])
        self.se = SegmentExtractor()
        self.se.to(self.device)

    def extract(self, key, img, **kwargs):
        return getattr(self, key)(img, **kwargs)

    @torch.no_grad()
    def dino_slic(self, img, n_segments=100, compactness=100.0, return_centers=False, return_image=False, show=False):
        """Extract dino feature form image

        Args:
            img (torch.Tensor, dtype=torch.float32, shape=(BS, C, H, W)): Image
            n_segments (int, optional): Defaults to 100.
            compactness (float, optional): Defaults to 100.0.
            return_centers (bool, optional): Defaults to False.
            return_image (bool, optional): Defaults to False.
            show (bool, optional): Defaults to False.

        Returns:
            adj (torch.Tensor, dtype=torch.uint32, shape=(N,2)): Graph Structure
            feat (torch.Tensor, dtype=torch.float32, shape=(N, C)): Features
            seg (torch.Tensor, dtype=torch.uint32, shape=(BS, 1, H, W)): Segmentation
            center (torch.Tensor, dtype=torch.float32, shape=(N,2)): Center of custer in image plane
        """

        # currently on BS=1 supported
        assert img.shape[0] == 1 and len(img.shape) == 4

        # prepare input image
        img = self.crop(img.to(self.device))
        img_store = img.clone()
        img = self.norm(img)

        # extract dino features
        feat_dino = self.si.model.net(img)[1]
        feat_dino = F.interpolate(feat_dino, img.shape[-2:], mode="bilinear", align_corners=False)

        # get slic clusters
        img_np = kornia.utils.tensor_to_image(img_store)
        seg = skimage.segmentation.slic(img_np, n_segments=n_segments, compactness=compactness, start_label=0)

        # extract adjacency_list based on clusters
        seg = torch.from_numpy(seg).to(self.device)[None, None]
        adjacency_list = self.se.adjacency_list(seg)

        # get mean dino features for each cluster
        features = []
        for i in range(seg.max() + 1):
            m = seg[0, 0] == i
            x, y = torch.where(m)
            feat = feat_dino[0, :, x, y].mean(dim=1)
            features.append(feat)

        ret = (adjacency_list, torch.stack(features, dim=1)[None], seg)

        if return_centers:
            ret += (self.se.centers(seg),)

        if show or return_image:
            ph = PlotHelper()
            ph.add(img_np, "Input Image Cropped")
            ph.add(seg, "SLIC Segmentation")

            img_pil = Image.fromarray(np.uint8(img_np * 255))
            seg_pil = Image.fromarray(np.uint8(seg.cpu().numpy()[0, 0]))
            img_draw = ImageDraw.Draw(img_pil)
            seg_draw = ImageDraw.Draw(seg_pil)

            centers = self.se.centers(seg)[0]
            fil_col = (seg.max() + 5).item()
            for i in range(adjacency_list.shape[1]):
                a, b = adjacency_list[0, i, 0], adjacency_list[0, i, 1]
                line_params = centers[a].tolist() + centers[b].tolist()
                img_draw.line(line_params, fill=fil_col)
                seg_draw.line(line_params, fill=fil_col)

            for i in range(centers.shape[0]):
                params = centers[i].tolist()
                params = [p - 2 for p in params] + [p + 2 for p in params]
                img_draw.ellipse(params, width=10, fill=fil_col + 1)
                seg_draw.ellipse(params, width=10, fill=fil_col + 1)

            ph.add(np.array(img_pil), "Image Feature-Graph")
            ph.add(np.array(seg_pil), "SLIC Feature-Graph")

            if show:
                ph.show()

            if return_image:
                ret += (ph.get_img(),)

        return ret

    def stego(self, img):
        return self.si.inference(img)
