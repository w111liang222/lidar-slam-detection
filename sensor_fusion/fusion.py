from module.detect.detect_template import DetectTemplate

class Fusion(DetectTemplate):
    def __init__(self, logger = None):
        super().__init__(name = 'fusion', logger = logger)

    def prepare(self):
        self.object_data = []
        self.trafficlight_data = []

    def do_fusion(self, object_data, trafficlight_data):
        fusion_data = {**object_data, **trafficlight_data}
        return self.post_process(fusion_data)

    def only_process_object(self, object_data):
        return self.post_process(object_data)

    def only_process_trafficlight(self, trafficlight_data):
        return self.post_process(trafficlight_data)

    def post_process(self, data):
        data.pop('detection_capability')
        return data

    def process(self, input_dict):
        if 'object' in input_dict:
            self.object_data.append(input_dict['object'])
        elif 'trafficlight' in input_dict:
            self.trafficlight_data.append(input_dict['trafficlight'])
        else:
            return dict()
        while len(self.object_data) > 0 and len(self.trafficlight_data) > 0:
            object_data = self.object_data[0]
            trafficlight_data = self.trafficlight_data[0]
            if object_data['frame_start_timestamp'] == trafficlight_data['frame_start_timestamp']:
                self.object_data.remove(object_data)
                self.trafficlight_data.remove(trafficlight_data)
                return self.do_fusion(object_data, trafficlight_data)
            elif object_data['frame_start_timestamp'] > trafficlight_data['frame_start_timestamp']:
                self.trafficlight_data.remove(trafficlight_data)
                return self.only_process_trafficlight(trafficlight_data)
            elif object_data['frame_start_timestamp'] < trafficlight_data['frame_start_timestamp']:
                self.object_data.remove(object_data)
                return self.only_process_object(object_data)

        if len(self.object_data) > 0:
            object_data = self.object_data[0]
            if not object_data['detection_capability']['trafficlight']:
                self.object_data.remove(object_data)
                return self.only_process_object(object_data)
            else:
                return dict()
        if len(self.trafficlight_data) > 0:
            trafficlight_data = self.trafficlight_data[0]
            if not trafficlight_data['detection_capability']['object']:
                self.trafficlight_data.remove(trafficlight_data)
                return self.only_process_trafficlight(trafficlight_data)
            else:
                return dict()

