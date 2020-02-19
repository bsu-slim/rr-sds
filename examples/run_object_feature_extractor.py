from retico.modules.keras.object_features import KerasObjectFeatureExtractorModule
from retico.interop.zeromq.io import ZeroMQWriter
from retico.interop.zeromq.io import ZeroMQReader
from retico.interop.zeromq.io import WriterSingleton
from retico.interop.zeromq.io import ReaderSingleton
from retico.core.debug.console import DebugModule

writer_s = WriterSingleton('132.178.227.12', port='12346') # wittgenstein's IP
reader_s = ReaderSingleton('10.255.221.241', port='12347')

reader = ZeroMQReader(topic='object_boxes')
feature_extractor = KerasObjectFeatureExtractorModule()
writer = ZeroMQWriter(topic='object_features')
debug = DebugModule()

reader.subscribe(feature_extractor)
reader.subscribe(debug)
feature_extractor.subscribe(writer)

reader.run()
feature_extractor.run()
writer.run()
debug.run()

input()

reader.stop()
feature_extractor.stop()
writer.stop()
debug.stop()

