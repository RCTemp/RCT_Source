package org.ros.android.map_nav;

import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;

public class StringPublisher extends AbstractNodeMain {
	/*
	 * メッセージ型はrosjava_coreのコンパイル時にrosjava_messages以下のbuild/generated-srcに生成されるので,
	 * その中から適宜使いたいメッセージ型を選択して使用する. 
	 * rosjava_coreビルド時にここにほぼ全てのメッセージが生成されると考えてよい.
	 * static final java.lang.String _TYPE = "std_msgs/String"; など, String.javaに定義されている.
	*/
	public Publisher<std_msgs.String> publisher; // Publisherクラスにトピックのメッセージ型を付けて宣言
	public static final String nodename = "string_publisher" ; // ノードの名前

	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of(StringPublisher.nodename); // クラス変数のノード名を返す
	}

	@Override
	public void onStart(final ConnectedNode connectedNode) { // 開始時に起動
		// connectedNode.newServiceClient
		this.publisher = connectedNode.newPublisher(StringPublisher.nodename + "/test",
				std_msgs.String._TYPE); // ノード名とトピック名を付けてROSネットワークに参加.
	}

	public void echo(String msg) {
		std_msgs.String str = this.publisher.newMessage(); // トピックにメッセージをセット
		str.setData(msg); // メッセージにデータをセット
		this.publisher.publish(str); // トピックをパブリッシュ
	}
}