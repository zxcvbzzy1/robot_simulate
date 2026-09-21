from fr5v6_weld_planning.tcp_config import sync_tcp_urdf


def test_tcp_yaml_values_replace_only_weld_tcp_origin(tmp_path):
    urdf = tmp_path / 'robot.urdf'
    urdf.write_text(
        '<robot><joint name="other"><origin xyz="9 9 9" rpy="9 9 9"/></joint>'
        '<joint name="tool0_to_weld_tcp" type="fixed">'
        '<origin xyz="0 0 0" rpy="0 0 0"/></joint></robot>', encoding='utf-8')
    sync_tcp_urdf({'tcp': {'xyz': [0.1, 0.2, 0.3], 'rpy': [0.4, 0.5, 0.6]}}, urdf)
    text = urdf.read_text()
    assert 'xyz="0.1 0.2 0.3" rpy="0.4 0.5 0.6"' in text
    assert 'xyz="9 9 9" rpy="9 9 9"' in text
