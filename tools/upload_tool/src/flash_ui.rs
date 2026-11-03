//! Device selection and flashing, ported from `upload_tool/FlashUI.cpp`.

use std::io::Write;
use std::sync::Arc;

use anyhow::{Result, bail};
use titan_tools::binary_info::AppInfo;
use titan_tools::canmore::Discovery;
use titan_tools::device_map::DeviceMap;
use titan_tools::flash::{Device, EraseWrapper, FLASH_BASE, FlashInterface};
use titan_tools::image::FlashImage;
use titan_tools::ui::{
    COLOR_BODY, COLOR_ID, COLOR_INFO_HEADER as COLOR_HEADER, COLOR_INFO_NAME as COLOR_NAME, COLOR_RESET, COLOR_TITLE,
    confirm, format_app_info, read_line,
};

pub fn dump_image(image: &FlashImage) {
    for app in &image.apps {
        let kind = if app.is_bootloader { "Bootloader" } else { "Application" };
        println!("{COLOR_TITLE}=========={kind} Image=========={COLOR_RESET}");
        print!("{}", format_app_info(app));
        println!();
    }
}

fn image_name(app: &AppInfo) -> String {
    let mut out = String::new();
    if app.is_bootloader {
        out += &format!("{COLOR_HEADER}[Bootloader] {COLOR_RESET}");
    }
    if app.program_name.is_empty() {
        out += &format!("{COLOR_HEADER}(No Program Name){COLOR_RESET}");
    } else {
        out += &format!("{COLOR_BODY}{}{COLOR_RESET}", app.program_name);
    }
    if !app.program_version.is_empty() {
        out += &format!(" {COLOR_BODY}{}{COLOR_RESET}", app.program_version);
    }
    out
}

fn print_device(index: usize, dev: &mut dyn Device, device_map: &DeviceMap) {
    let descr = device_map.lookup_serial(dev.flash_id());
    if index != 0 {
        print!("{COLOR_ID}Dev {index}:\t");
    } else {
        print!("\t");
    }
    println!("{COLOR_HEADER}Name:\t\t{COLOR_NAME}{}{COLOR_RESET}", descr.name);
    println!("{COLOR_HEADER}\tInterface:\t{COLOR_BODY}{}{COLOR_RESET}", dev.interface());
    println!("{COLOR_HEADER}\tBoard Type:\t{COLOR_BODY}{}{COLOR_RESET}", descr.board_type);
    println!("{COLOR_HEADER}\tUnique ID:\t{COLOR_BODY}{}{COLOR_RESET}", descr.hex_serial());
    for (label, value) in dev.additional_info() {
        let tab = if label.len() <= 6 { "\t" } else { "" };
        println!("{COLOR_HEADER}\t{label}:\t{tab}{COLOR_BODY}{value}{COLOR_RESET}");
    }
    println!();
}

/// Picks a device: the only flashable device matching `board_type` if `auto_select`, otherwise by asking.
/// Returns the index into `devices`, or `None` if the user aborted.
pub fn select_device(
    devices: &mut [Box<dyn Device>],
    device_map: &DeviceMap,
    board_type: &str,
    auto_select: bool,
) -> Option<usize> {
    let board_types: Vec<String> =
        devices.iter_mut().map(|d| device_map.lookup_serial(d.flash_id()).board_type).collect();

    if auto_select {
        let mut matching = (0..devices.len()).filter(|&i| board_types[i] == board_type);
        // The flash interface check comes after the uniqueness check, so a matching board in a mode that can't be
        // flashed isn't hidden from the user
        if let (Some(only), None) = (matching.next(), matching.next())
            && devices[only].supports_flash_interface()
        {
            println!("{COLOR_TITLE}Auto Selecting Device:{COLOR_RESET}");
            print_device(0, devices[only].as_mut(), device_map);
            return Some(only);
        }
    }

    // Compatible (same board type), other flashable, and non-flashable devices, each sorted by interface
    let mut good = Vec::new();
    let mut other = Vec::new();
    let mut invalid = Vec::new();
    for (i, dev) in devices.iter().enumerate() {
        if !dev.supports_flash_interface() {
            invalid.push(i);
        } else if board_types[i] == board_type {
            good.push(i);
        } else {
            other.push(i);
        }
    }
    for list in [&mut good, &mut other, &mut invalid] {
        list.sort_by_key(|&i| devices[i].interface());
    }

    let mut index = 1;
    if !good.is_empty() {
        println!("{COLOR_TITLE}===========Compatible Targets==========={COLOR_RESET}");
        for &i in &good {
            print_device(index, devices[i].as_mut(), device_map);
            index += 1;
        }
    }
    if !other.is_empty() {
        println!("{COLOR_TITLE}=============Other Targets============={COLOR_RESET}");
        for &i in &other {
            print_device(index, devices[i].as_mut(), device_map);
            index += 1;
        }
    }
    if !invalid.is_empty() {
        println!("{COLOR_TITLE}=========Non-Flashable Targets========={COLOR_RESET}");
        for &i in &invalid {
            print_device(0, devices[i].as_mut(), device_map);
        }
    }

    loop {
        let Some(input) = read_line("Please Select a Device [none to abort]: ") else {
            println!("\nNo stdin available! Aborting upload...");
            return None;
        };
        if input.is_empty() {
            println!("Upload aborted...");
            return None;
        }
        let choice = match input.parse::<usize>() {
            Ok(c) if (1..index).contains(&c) => c - 1,
            _ => {
                println!("Invalid input!");
                continue;
            }
        };
        if choice < good.len() {
            return Some(good[choice]);
        }
        print!("[WARNING] The selected board does not match the expected type '{board_type}'. ");
        if confirm("Continue?") {
            return Some(other[choice - good.len()]);
        }
    }
}

/// Picks the flashable device with flash ID `serial`.
pub fn select_device_by_serial(devices: &mut [Box<dyn Device>], device_map: &DeviceMap, serial: u64) -> Result<usize> {
    for (i, dev) in devices.iter_mut().enumerate() {
        if dev.flash_id() == serial && dev.supports_flash_interface() {
            print_device(0, dev.as_mut(), device_map);
            return Ok(i);
        }
    }
    bail!("Could not find requested device: {}", device_map.lookup_serial(serial).name)
}

/// Asks for a CANmore interface and client ID, then waits for that device to boot and stops it in the bootloader.
pub fn catch_in_boot_delay(
    sources: &[Arc<Discovery>],
    device_map: &DeviceMap,
    image: &FlashImage,
) -> Result<Box<dyn FlashInterface>> {
    if sources.is_empty() {
        bail!("No CANmore interfaces available");
    }
    println!("{COLOR_TITLE}===========Available Interfaces==========={COLOR_RESET}");
    for (i, source) in sources.iter().enumerate() {
        println!("Itf {}:\tName:\t{COLOR_NAME}{}{COLOR_RESET}", i + 1, source.interface_name());
    }

    let interface = loop {
        let Some(input) = read_line("\nPlease Select a Interface: ") else { bail!("No stdin available") };
        match input.parse::<usize>() {
            Ok(c) if (1..=sources.len()).contains(&c) => break &sources[c - 1],
            _ if input.is_empty() => continue,
            _ => println!("Invalid input!"),
        }
    };

    // Suggest the client ID the image was built for
    let mut default_id = None;
    for app in &image.apps {
        if !app.client_ids.is_empty() && !app.device_ip_address.is_empty() {
            println!("Both Ethernet and CAN bus supported?? Unable to determine image client id");
        } else if !app.device_ip_address.is_empty() {
            println!("Found ethernet image with IP address: {}", app.device_ip_address);
            println!("   - The client ID for ethernet interfaces is the last byte of the bootloader IP address.");
            break;
        } else if let [id] = app.client_ids.as_slice() {
            match u8::try_from(*id) {
                Ok(id) if id != 0 && id != 255 => default_id = Some(id),
                _ => println!("Invalid client ID decoded from image: {id}"),
            }
            break;
        } else if app.client_ids.len() > 1 {
            println!("Provided Firmware Image has multiple valid CAN IDs:");
            for id in &app.client_ids {
                println!("   - {id}");
            }
            break;
        }
    }

    let client_id = loop {
        let prompt = match default_id {
            Some(id) => format!("Enter client id to wait for [Detected Default={id}]: "),
            None => "Enter client id to wait for: ".to_owned(),
        };
        let Some(input) = read_line(&prompt) else { bail!("No stdin available") };
        if input.is_empty() {
            if let Some(id) = default_id {
                println!("Selecting default id {id}");
                break id;
            }
            continue;
        }
        match input.parse::<u8>() {
            Ok(id) if id != 0 && id != 255 => break id,
            _ => println!("Invalid input!"),
        }
    };

    println!("Waiting for device...");
    let mut client = interface.catch_in_boot_delay(client_id)?;
    for warning in &client.warnings {
        println!("{warning}");
    }
    let descr = device_map.lookup_serial(client.flash_id());
    println!("{COLOR_TITLE}Found Device:{COLOR_RESET}");
    println!("{COLOR_HEADER}\tName:\t\t{COLOR_NAME}{}{COLOR_RESET}", descr.name);
    println!("{COLOR_HEADER}\tInterface:\t{COLOR_BODY}{}{COLOR_RESET}", interface.interface_name());
    println!("{COLOR_HEADER}\tBoard Type:\t{COLOR_BODY}{}{COLOR_RESET}", descr.board_type);
    println!("{COLOR_HEADER}\tUnique ID:\t{COLOR_BODY}{}{COLOR_RESET}", descr.hex_serial());
    println!("{COLOR_HEADER}\tBL Version:\t{COLOR_BODY}{}{COLOR_RESET}", client.version().unwrap_or_default());
    println!();

    if descr.board_type != image.board_type {
        print!("[WARNING] The discovered board does not match the expected type '{}'. ", image.board_type);
        if !confirm("Continue?") {
            bail!("Flash Aborted");
        }
    }
    Ok(Box::new(client))
}

fn draw_progress(fraction: f32) {
    const WIDTH: usize = 40;
    let fraction = fraction.clamp(0.0, 1.0);
    let pos = (WIDTH as f32 * fraction) as usize;
    let bar: String = (0..WIDTH)
        .map(|i| {
            if i < pos {
                '='
            } else if i == pos {
                '>'
            } else {
                ' '
            }
        })
        .collect();
    print!("[{bar}] {} %\r", (fraction * 100.0) as u32);
    let _ = std::io::stdout().flush();
}

/// Writes `image` through `interface`, verifies it and reboots. `is_ota` restricts the image to the application
/// region after the bootloader. Returns false if the user aborted or verification failed.
pub fn flash_image(interface: &mut dyn FlashInterface, image: &FlashImage, is_ota: bool) -> Result<bool> {
    let offset = image.base_flash_offset();
    let bootloader_size = interface.bootloader_size();

    if offset as u64 + image.size() as u64 > interface.flash_size() as u64 {
        println!(
            "[ERROR] Requested image @+{offset:#x} len {:#x} is too large to fit into device flash size {:#x}",
            image.size(),
            interface.flash_size()
        );
        return Ok(false);
    }

    if is_ota && offset < bootloader_size {
        println!(
            "[ERROR] Requested OTA image @+{offset:#x} overlaps with bootloader reserved space @+{bootloader_size:#x}"
        );
        println!("Use the -f flag if you want to flash a full uf2, rather than just an ota.");
        return Ok(false);
    }

    if !is_ota && interface.warn_on_bootloader_overwrite() {
        if !confirm(
            "[WARNING!] Attempting to flash/overwrite the bootloader via the bootloader interface! This may brick the \
             interface!\nAre you CERTAIN you want to continue flashing the full image?",
        ) {
            println!("Flash aborted.");
            return Ok(false);
        }
        interface.enable_bootloader_overwrite()?;
    }

    if is_ota
        && offset != 0
        && bootloader_size == 0
        && !confirm("[WARNING] Attempting to flash OTA image but no bootloader was found. Continue with flashing?")
    {
        println!("Flash aborted.");
        return Ok(false);
    }

    // An application image starting with its own boot2 should carry the same boot2 as the bootloader, since the
    // SDK calls whatever boot2 sits at XIP_BASE
    if is_ota
        && offset != 0
        && offset == bootloader_size
        && !interface.verify_page(FLASH_BASE, image.page(0))?
        && !confirm(
            "[WARNING] The boot2 used by this application does not match the bootloader boot2. This may cause \
             performance or other stability issues. Continue with flashing?",
        )
    {
        println!("Flash aborted.");
        return Ok(false);
    }

    let full = !(is_ota && offset != 0);
    print!("\nFlashing {} Image:", if full { "Full" } else { "OTA" });
    match image.apps.as_slice() {
        [] => println!("{COLOR_HEADER} (No Binary Info Found){COLOR_RESET}"),
        [app] => println!(" {}", image_name(app)),
        apps => {
            println!();
            for app in apps {
                println!(" - {}", image_name(app));
            }
        }
    }
    println!();

    // Write everything but the first page first. Erasing works in 4 KiB sectors, so the first page (boot2, which
    // the bootrom/bootloader checks before booting) is erased along with the second page. If the upload is
    // interrupted, the previous stage sees an invalid boot2 and waits for a new image instead of booting garbage.
    let mut wrapper = EraseWrapper::new(interface);
    let count = image.num_pages();
    println!("Writing Image...");
    for i in 1..count {
        draw_progress(i as f32 / (count - 1) as f32);
        wrapper.write_page(image.page_address(i), image.page(i))?;
    }
    draw_progress(1.0);

    println!("\nVerifying Image...");
    for i in 1..count {
        draw_progress(i as f32 / (count - 1) as f32);
        if !wrapper.interface().verify_page(image.page_address(i), image.page(i))? {
            println!("\nVerify failed at address {:#010x}", image.page_address(i));
            return Ok(false);
        }
    }
    draw_progress(1.0);
    println!();

    println!("Sealing Image...");
    wrapper.write_page(image.page_address(0), image.page(0))?;
    if !wrapper.interface().verify_page(image.page_address(0), image.page(0))? {
        println!("Verify failed at address {:#010x}", image.page_address(0));
        return Ok(false);
    }

    println!("Image Written.");
    wrapper.interface().reboot()?;
    Ok(true)
}

#[cfg(test)]
mod tests {
    use super::*;
    use titan_tools::flash::{FLASH_ERASE_SIZE, PAGE_SIZE, Page};

    /// A flash chip in memory, recording the operations.
    struct FakeFlash {
        mem: Vec<u8>,
        bootloader_size: u32,
        erases: Vec<u32>,
        writes: Vec<u32>,
        rebooted: bool,
    }

    impl FakeFlash {
        fn new(size: usize, bootloader_size: u32) -> Self {
            FakeFlash { mem: vec![0xAA; size], bootloader_size, erases: vec![], writes: vec![], rebooted: false }
        }
    }

    impl FlashInterface for FakeFlash {
        fn erase_sector(&mut self, addr: u32) -> Result<()> {
            assert_eq!(addr % FLASH_ERASE_SIZE, 0);
            let off = (addr - FLASH_BASE) as usize;
            self.mem[off..off + FLASH_ERASE_SIZE as usize].fill(0xFF);
            self.erases.push(addr);
            Ok(())
        }
        fn read_page(&mut self, addr: u32) -> Result<Page> {
            let off = (addr - FLASH_BASE) as usize;
            Ok(self.mem[off..off + PAGE_SIZE].try_into().unwrap())
        }
        fn write_page(&mut self, addr: u32, data: &Page) -> Result<()> {
            let off = (addr - FLASH_BASE) as usize;
            assert!(self.mem[off..off + PAGE_SIZE].iter().all(|&b| b == 0xFF), "writing unerased page {addr:#x}");
            self.mem[off..off + PAGE_SIZE].copy_from_slice(data);
            self.writes.push(addr);
            Ok(())
        }
        fn bootloader_size(&self) -> u32 {
            self.bootloader_size
        }
        fn flash_id(&self) -> u64 {
            0
        }
        fn flash_size(&self) -> u32 {
            self.mem.len() as u32
        }
        fn reboot(&mut self) -> Result<()> {
            self.rebooted = true;
            Ok(())
        }
        fn warn_on_bootloader_overwrite(&self) -> bool {
            false
        }
    }

    fn image(base: u32, pages: usize) -> FlashImage {
        let pages: Vec<Page> = (0..pages).map(|i| [i as u8; PAGE_SIZE]).collect();
        FlashImage::new(base, pages).unwrap()
    }

    #[test]
    fn full_image_is_written_sealed_last() {
        let mut flash = FakeFlash::new(64 * 1024, 0);
        let image = image(FLASH_BASE, 40);
        assert!(flash_image(&mut flash, &image, true).unwrap());
        assert_eq!(flash.writes.len(), 40);
        assert_eq!(*flash.writes.last().unwrap(), FLASH_BASE, "first page must be written last");
        assert_eq!(flash.erases, [FLASH_BASE, FLASH_BASE + 0x1000, FLASH_BASE + 0x2000]);
        for i in 0..40 {
            assert!(flash.mem[i * PAGE_SIZE..(i + 1) * PAGE_SIZE].iter().all(|&b| b == i as u8));
        }
        assert!(flash.rebooted);
    }

    #[test]
    fn rejects_bad_images() {
        // Too large for the flash
        let mut flash = FakeFlash::new(8 * 1024, 0);
        assert!(!flash_image(&mut flash, &image(FLASH_BASE, 40), true).unwrap());
        // OTA image overlapping the bootloader
        let mut flash = FakeFlash::new(64 * 1024, 0x4000);
        assert!(!flash_image(&mut flash, &image(FLASH_BASE + 0x1000, 4), true).unwrap());
        assert!(flash.writes.is_empty() && !flash.rebooted);
    }

    #[test]
    fn ota_image_after_bootloader() {
        let mut flash = FakeFlash::new(64 * 1024, 0x4000);
        // The bootloader's boot2 must match the image's first page to avoid a prompt
        flash.mem[..PAGE_SIZE].fill(0);
        assert!(flash_image(&mut flash, &image(FLASH_BASE + 0x4000, 20), true).unwrap());
        assert_eq!(flash.erases, [FLASH_BASE + 0x4000, FLASH_BASE + 0x5000]);
        assert_eq!(*flash.writes.last().unwrap(), FLASH_BASE + 0x4000);
    }
}
