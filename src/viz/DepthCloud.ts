import { DEFAULT_OPTIONS_DEPTHCLOUD } from '../utils/constants';
import DepthCloudObject from '../primitives/DepthCloudObject';
import LiveCore from '../core/live';

class DepthCloud extends LiveCore<{}, DepthCloudObject> {
 

  constructor(url: string, options = DEFAULT_OPTIONS_DEPTHCLOUD) {
    super({
      sources: [],
      options: {
        ...DEFAULT_OPTIONS_DEPTHCLOUD,
        ...options,
      },
    });
    
    this.object = new DepthCloudObject(url, options);
  }

  subscribe = () => {
    this.object?.initVideo();
    this.object?.startStream();
  };

  unsubscribe = () => {
    this.object?.stopStream();
  };
}

export default DepthCloud;
