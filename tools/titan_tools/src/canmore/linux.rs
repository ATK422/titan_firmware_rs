//! Client for the debug daemon on Linux machines, ported from `canmore_dbg_cpp/LinuxClient.cpp`.

use anyhow::{Result, bail, ensure};
use canmore::ControlInterfaceMode;
use canmore::reg_mapped::PAGE_SIZE;
use canmore::reg_mapped::interface::linux::{self as itf, gen_control, tty_control};

use super::{RegClient, flash_id_from_words};

/// Client for the Linux interface ([`ControlInterfaceMode::Linux`]).
pub struct LinuxClient {
    client: RegClient,
    flash_id: u64,
    link: Option<(super::Link, u8)>,
}

impl LinuxClient {
    pub fn new(mut client: RegClient) -> Result<Self> {
        let magic = client.read(itf::GEN_CONTROL_PAGE, gen_control::MAGIC)?;
        if magic != gen_control::MAGIC_VALUE {
            bail!("Unexpected value in general control magic for linux: 0x{magic:08x}");
        }
        let lower = client.read(itf::GEN_CONTROL_PAGE, gen_control::LOWER_FLASH_ID)?;
        let upper = client.read(itf::GEN_CONTROL_PAGE, gen_control::UPPER_FLASH_ID)?;
        Ok(LinuxClient { client, flash_id: flash_id_from_words(lower, upper), link: None })
    }

    pub fn open(link: &super::Link, client_id: u8) -> Result<Self> {
        let mut client = Self::new(RegClient::open(link, client_id, ControlInterfaceMode::Linux)?)?;
        client.link = Some((link.clone(), client_id));
        Ok(client)
    }

    /// The link and client ID the client was opened on (needed for the remote TTY and camera feed channels).
    pub fn link(&self) -> Option<(&super::Link, u8)> {
        self.link.as_ref().map(|(link, id)| (link, *id))
    }

    pub fn flash_id(&self) -> u64 {
        self.flash_id
    }

    /// The underlying register client, for the file transfer pages.
    pub fn client(&mut self) -> &mut RegClient {
        &mut self.client
    }

    pub fn version(&mut self) -> Result<String> {
        self.client.read_string_page(itf::VERSION_STRING_PAGE)
    }

    /// Starts a remote TTY session running `cmd` (a login shell if empty) with `TERM=term_env`.
    pub fn enable_remote_tty(
        &mut self,
        term_env: &str,
        rows: u16,
        cols: u16,
        cmd: &str,
        use_upload_dir: bool,
    ) -> Result<()> {
        ensure!(term_env.len() < PAGE_SIZE, "Terminal environment variable too large to transmit");
        ensure!(cmd.len() < PAGE_SIZE, "Command too large to transmit");
        self.client.write_string_page(itf::TTY_TERMINAL_PAGE, term_env.as_bytes())?;
        self.client.write_string_page(itf::TTY_CMD_PAGE, cmd.as_bytes())?;
        let page = itf::TTY_CONTROL_PAGE;
        self.client.write(page, tty_control::WINDOW_SIZE, (rows as u32) << 16 | cols as u32)?;
        self.client.write(page, tty_control::USE_UPLOAD_DIR, use_upload_dir as u32)?;
        self.client.write(page, tty_control::ENABLE, 1)
    }

    pub fn disable_remote_tty(&mut self) -> Result<()> {
        self.client.write(itf::TTY_CONTROL_PAGE, tty_control::ENABLE, 0)
    }

    pub fn remote_tty_enabled(&mut self) -> Result<bool> {
        Ok(self.client.read(itf::TTY_CONTROL_PAGE, tty_control::ENABLE)? != 0)
    }

    pub fn reboot(&mut self) -> Result<()> {
        self.client.write(itf::GEN_CONTROL_PAGE, gen_control::REBOOT_DEVICE, gen_control::REBOOT_DEVICE_MAGIC)
    }

    pub fn restart_daemon(&mut self) -> Result<()> {
        self.client.write(itf::GEN_CONTROL_PAGE, gen_control::RESTART_DAEMON, gen_control::RESTART_DAEMON_MAGIC)
    }

    pub fn ping(&mut self) -> Result<()> {
        let magic = self.client.read(itf::GEN_CONTROL_PAGE, gen_control::MAGIC)?;
        if magic != gen_control::MAGIC_VALUE {
            bail!("Unexpected value in general control magic for linux");
        }
        Ok(())
    }
}
